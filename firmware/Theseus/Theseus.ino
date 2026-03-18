//#include "MotorWheel.h" //some code reproduced
#include "PinChangeInt.h"
#include <PID_v1.h>

#define DIR_ADVANCE HIGH
#define DIR_BACKOFF LOW
#define MAX_PWM 255
#define PIN_UNDEFINED 255
#ifdef _NAMIKI_MOTOR
#define TRIGGER CHANGE
#define CPR 4  // Namiki motor
#define DIR_INVERSE
#define REDUCTION_RATIO 80
#else
#define TRIGGER RISING
#define CPR 12  // Faulhaber motor
#define DIR_INVERSE !
#define REDUCTION_RATIO 64
#endif
#define PULSESPREV 157
#define SEC_PER_MIN 60
#define MICROS_PER_SEC 1000000
#define SPEEDPPS2SPEEDRPM(freq) ((unsigned long)(freq) * (SEC_PER_MIN) / (CPR))

#define irqISR(y, x) \
  void x(); \
  struct ISRVars y = { x }; \
  void x() { \
    static bool first_pulse = true; \
    y.pulseEndMicros = micros(); \
    if (first_pulse == false && y.pulseEndMicros > y.pulseStartMicros) { \
      y.speedPPS = MICROS_PER_SEC / (y.pulseEndMicros - y.pulseStartMicros); \
      /* y.accPPSS=(y.speedPPS-y.lastSpeedPPS)*y.speedPPS; */ \
    } else first_pulse = false; \
    y.pulseStartMicros = y.pulseEndMicros; \
    /* y.lastSpeedPPS=y.speedPPS; */ \
    if (y.pinIRQB != PIN_UNDEFINED) \
      y.currDirection = DIR_INVERSE(digitalRead(y.pinIRQ) ^ digitalRead(y.pinIRQB)); \
    y.currDirection == DIR_ADVANCE ? ++y.pulses : --y.pulses; \
  }

struct ISRVars {
  void (*ISRfunc)();
  //volatile unsigned long pulses;
  volatile long pulses;  // 201104, direction sensitive
  volatile unsigned long pulseStartMicros;
  volatile unsigned long pulseEndMicros;
  volatile unsigned int speedPPS;
  //volatile unsigned int  lastSpeedPPS;
  //volatile int accPPSS;	// acceleration, Pulse Per Sec^2
  volatile bool currDirection;
  unsigned char pinIRQB;
  unsigned char pinIRQ;  // pinIRQA 201207
};

#define W1_PWM 3
#define W1_DIR 2
#define W1_IRQ 14 //4
#define W1_IRQB 15 //5
#define W2_PWM 11
#define W2_DIR 12
#define W2_IRQ 4 //14
#define W2_IRQB 5 //15
#define W3_PWM 9
#define W3_DIR 8
#define W3_IRQ 18 //16
#define W3_IRQB 19 //17
#define W4_PWM 10
#define W4_DIR 7
#define W4_IRQ 16 //18
#define W4_IRQB 17 //19

class MotorHandler {
public:
  MotorHandler(bool rev_, unsigned char _pinPWM, unsigned char _pinDir,
               unsigned char _pinIRQ, unsigned char _pinIRQB,
               struct ISRVars* _isr)
    : pinPWM(_pinPWM), pinDir(_pinDir), isr(_isr), reverse(rev_), wheelPID(&measuredRADPS, &PIDOut, &desiredRADPS, 0.31, 0.01, 0.0, DIRECT) {

    isr->pinIRQ = _pinIRQ;
    isr->pinIRQB = _pinIRQB;
  }

  void setup() {
    pinMode(pinPWM, OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(isr->pinIRQ, INPUT);

    if (isr->pinIRQB != PIN_UNDEFINED) {
      pinMode(isr->pinIRQB, INPUT);
    }

    if (isr->pinIRQ == 2 || isr->pinIRQ == 3) attachInterrupt(isr->pinIRQ - 2, isr->ISRfunc, TRIGGER);
    else {
      PCattachInterrupt(isr->pinIRQ, isr->ISRfunc, TRIGGER);  // RISING --> CHANGE 201207
    }

    wheelPID.SetMode(AUTOMATIC);
    wheelPID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  }

  double getDesiredRADPS() const {
    //return (float)GearedMotor::getGearedSpeedRPM() / radPsToRPM * ((reverse) ? -1 : 1);
    return desiredRADPS;
  }
  double setDesiredRADPS(float radPs) {
    //GearedMotor::setGearedSpeedRPM(radPs * radPsToRPM * ((reverse) ? -1 : 1));
    desiredRADPS = radPs;
    return getDesiredRADPS();
  }
  double getMeasuredRADPS() {                      //WORKING but left side reports the other way around
    if (micros() - isr->pulseEndMicros > 10000) {  //speedPPS doesn't reset so detect if measure is stale
      return 0.0;
    }
    //CPR should be the number of ticks per revolution
    double RADPS = (isr->speedPPS / (double)PULSESPREV) * 2 * PI;  //(encPulses/s) * (1 rev/PULSESPREV encPulses) * (2PI rads/rev) = rads/s
    if (getCurrDir() == ((reverse) ? DIR_ADVANCE : DIR_BACKOFF)) {
      RADPS = -RADPS;
    }
    // if (reverse) {
    //   RADPS = -RADPS;
    // }
    measuredRADPS = RADPS;
    return measuredRADPS;
  }
  double getMeasuredRPS() {                      //WORKING but left side reports the other way around
    if (micros() - isr->pulseEndMicros > 10000) {  //speedPPS doesn't reset so detect if measure is stale
      return 0.0;
    }
    double RPS = (isr->speedPPS / (double)PULSESPREV);  //(encPulses/s) * (1 rev/PULSESPREV encPulses) = rev/s
    if (getCurrDir() == ((reverse) ? DIR_ADVANCE : DIR_BACKOFF)) {
      RPS = -RPS;
    }
    return RPS;
  }
  volatile long getPulseCount() {
    return isr->pulses;
  }
  volatile long getSpeedPPS() {
    return isr->speedPPS;
  }
  volatile bool getCurrDir() const {
    return isr->currDirection;
  }
  void PIDSet(double Kp, double Ki, double Kd, int sampleMs = 20) {
    wheelPID.SetSampleTime(sampleMs);
    wheelPID.SetTunings(Kp, Ki, Kd);
  }
  bool Compute() {
    getMeasuredRADPS();
    if (wheelPID.Compute()) {    //new output (flCMD>=0)?DIR_ADVANCE:DIR_BACKOFF (rrCMD<0)?DIR_ADVANCE:DIR_BACKOFF
      // bool dir = ((reverse) ? (PIDOut >= 0) : !(PIDOut >= 0));
      bool dir;
      if (reverse) {
        dir = (PIDOut>=0)?DIR_ADVANCE:DIR_BACKOFF;
      } else {
        dir = (PIDOut<0)?DIR_ADVANCE:DIR_BACKOFF;
      }
      double processedOut = (abs(PIDOut) <= MAX_PWM) ? abs(PIDOut) : MAX_PWM;
      analogWrite(pinPWM, processedOut);
      digitalWrite(pinDir, dir);
      // fl_wheel.runPWM(flCMD,(flCMD>=0)?DIR_ADVANCE:DIR_BACKOFF);
      // rl_wheel.runPWM(rlCMD,(rlCMD>=0)?DIR_ADVANCE:DIR_BACKOFF);
      // rr_wheel.runPWM(rrCMD,(rrCMD<0)?DIR_ADVANCE:DIR_BACKOFF);
      // fr_wheel.runPWM(frCMD,(frCMD<0)?DIR_ADVANCE:DIR_BACKOFF);
      //Serial.print("Running ");Serial.println(processedOut);
      return true;
    } else {
      return false;
    }
  }
  void reportPID() {
    Serial.print("Measured:");Serial.print(measuredRADPS);Serial.print("; Desired:");Serial.print(desiredRADPS);Serial.print("; PIDOut:");Serial.println(PIDOut);
  }
  unsigned int runPWM(unsigned int PWM,bool dir) {
    analogWrite(pinPWM,PWM);
    digitalWrite(pinDir,dir);
    return PWM;
  }

  double PIDOut;
private:
  //double radPsToRPM = SEC_PER_MIN / (2 * PI);
  struct ISRVars* isr;
  unsigned char pinPWM;
  unsigned char pinDir;
  bool reverse;
  volatile double desiredRADPS;
  volatile double measuredRADPS;
  PID wheelPID;
};

irqISR(irq1, isr1);
MotorHandler rl_wheel(1, W1_PWM, W1_DIR, W1_IRQ, W1_IRQB, &irq1);

irqISR(irq2, isr2);
MotorHandler fl_wheel(1, W2_PWM, W2_DIR, W2_IRQ, W2_IRQB, &irq2);

irqISR(irq3, isr3);
MotorHandler fr_wheel(0, W3_PWM, W3_DIR, W3_IRQ, W3_IRQB, &irq3);

irqISR(irq4, isr4);
MotorHandler rr_wheel(0, W4_PWM, W4_DIR, W4_IRQ, W4_IRQB, &irq4);

void setDesiredVelocities(MotorHandler* fl, MotorHandler* rl, MotorHandler* rr, MotorHandler* fr, float flCMD, float rlCMD, float rrCMD, float frCMD) {
  fl->setDesiredRADPS(flCMD);
  rl->setDesiredRADPS(rlCMD);
  rr->setDesiredRADPS(rrCMD);
  fr->setDesiredRADPS(frCMD);
}
//should be the same as sendDesiredVelocities()
size_t sendEncoderVelocities(MotorHandler* fl, MotorHandler* rl, MotorHandler* rr, MotorHandler* fr) {
  size_t count = Serial.print("e:");
  count += Serial.print(fl->getMeasuredRADPS());
  count += Serial.print(",");
  count += Serial.print(fr->getMeasuredRADPS());
  count += Serial.print(",");
  count += Serial.print(rl->getMeasuredRADPS());
  count += Serial.print(",");
  count += Serial.println(rr->getMeasuredRADPS());
  return count;
}

size_t sendDesiredVelocities(MotorHandler* fl, MotorHandler* rl, MotorHandler* rr, MotorHandler* fr) {
  size_t count = Serial.print("vs:");
  count += Serial.print(fl->getDesiredRADPS());
  count += Serial.print(",");
  count += Serial.print(fr->getDesiredRADPS());
  count += Serial.print(",");
  count += Serial.print(rl->getDesiredRADPS());
  count += Serial.print(",");
  count += Serial.println(rr->getDesiredRADPS());
  return count;
}

void PIDSet(MotorHandler* fl, MotorHandler* rl, MotorHandler* rr, MotorHandler* fr, float Kp, float Ki, float Kd, int sampleMs = 20) {
  fl->PIDSet(Kp, Ki, Kd, sampleMs);
  rl->PIDSet(Kp, Ki, Kd, sampleMs);
  rr->PIDSet(Kp, Ki, Kd, sampleMs);
  fr->PIDSet(Kp, Ki, Kd, sampleMs);
}

void ComputePIDs(MotorHandler* fl, MotorHandler* rl, MotorHandler* rr, MotorHandler* fr) {
  fl->Compute();
  rl->Compute();
  rr->Compute();
  fr->Compute();
}

void setup() {
  TCCR1B = TCCR1B & 0xf8 | 0x01;  // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01;  // Pin3,Pin11 PWM 31250Hz

  Serial.begin(115200);

  fl_wheel.setup();
  rl_wheel.setup();
  rr_wheel.setup();
  fr_wheel.setup();

  PIDSet(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel, 30, 2, 0.01, 20);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data == "e") {
      sendEncoderVelocities(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel);
    } else if (data.startsWith("v:")) {  //format: v:<fl>,<fr>,<rl>,<rr>\n`
      data.remove(0, 2);                 //delete v:
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);
      int thirdComma = data.indexOf(',', secondComma + 1);

      double flCMD = data.substring(0, firstComma).toDouble();
      double frCMD = data.substring(firstComma + 1, secondComma).toDouble();
      double rlCMD = data.substring(secondComma + 1, thirdComma).toDouble();
      double rrCMD = data.substring(thirdComma + 1, data.length()).toDouble();

      setDesiredVelocities(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel, flCMD, rlCMD, rrCMD, frCMD);
      //sendDesiredVelocities(&fl_wheel,&rl_wheel,&rr_wheel,&fr_wheel);
      // fl_wheel.runPWM(flCMD,(flCMD>=0)?DIR_ADVANCE:DIR_BACKOFF);
      // rl_wheel.runPWM(rlCMD,(rlCMD>=0)?DIR_ADVANCE:DIR_BACKOFF);
      // rr_wheel.runPWM(rrCMD,(rrCMD<0)?DIR_ADVANCE:DIR_BACKOFF);
      // fr_wheel.runPWM(frCMD,(frCMD<0)?DIR_ADVANCE:DIR_BACKOFF);
    } else if (data.startsWith("p")) {
      setDesiredVelocities(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel, 0, PI, 0, 0);
    } else if (data.startsWith("o")) {
      setDesiredVelocities(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel, 0, -PI, 0, 0);
    } else if (data.startsWith("s")) {
      setDesiredVelocities(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel, 0, 0, 0, 0);
    } else if (data.startsWith("z")) {
      sendDesiredVelocities(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel);
    } else if (data.startsWith("1")) {
      Serial.println(rl_wheel.getPulseCount());
    } else if (data.startsWith("2")) {
      Serial.println(rl_wheel.getSpeedPPS());
    } else if (data.startsWith("3")) {
      Serial.println(rl_wheel.PIDOut);
    } else if (data.startsWith("4")) {
      if (rl_wheel.getCurrDir() == DIR_BACKOFF) {
        Serial.println("Inverting");
      } else Serial.println("Not Inverting");
    } else if (data.startsWith("5")) {
      fl_wheel.runPWM(255, DIR_ADVANCE);
      //Serial.println(rl_wheel.getMeasuredRPS());
    }
  }
  //Serial.print("MeasuredRADPS:");Serial.print()
  ComputePIDs(&fl_wheel, &rl_wheel, &rr_wheel, &fr_wheel);
  //fl_wheel.getMeasuredRADPS();
  //fl_wheel.reportPID();
}