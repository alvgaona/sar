#include "MotorWheel.h"

#define W1_PWM 3
#define W1_DIR 2
#define W1_IRQ 4
#define W1_IRQB 5
#define W2_PWM 11
#define W2_DIR 12
#define W2_IRQ 14
#define W2_IRQB 15
#define W3_PWM 9
#define W3_DIR 8
#define W3_IRQ 16
#define W3_IRQB 17
#define W4_PWM 10
#define W4_DIR 7
#define W4_IRQ 18
#define W4_IRQB 19

class MotorHandler: public GearedMotor {
public:
	MotorHandler(unsigned char _pinPWM,unsigned char _pinDir,
					unsigned char _pinIRQ,unsigned char _pinIRQB,
					struct ISRVars* _isr,
					unsigned int _ratio=REDUCTION_RATIO):
          GearedMotor(_pinPWM,_pinDir,_pinIRQ,_pinIRQB,_isr,_ratio) {}

  float getDesiredRADPS() const {
    return (float)getGearedSpeedRPM()/radPsToRPM;
  }
  float setDesiredRADPS(float radPs){
    setGearedSpeedRPM(radPs*radPsToRPM);
	  return getDesiredRADPS();
  }
  //should be the same or faster as getDesiredRADPS()
  float getMeasuredRADPS(){
    int speedRPM;
    if(getPinIRQB()!=PIN_UNDEFINED && getDesiredDir()!=getCurrDir()) {
      speedRPM=-SPEEDPPS2SPEEDRPM(isr->speedPPS);
    } else {
      speedRPM=SPEEDPPS2SPEEDRPM(isr->speedPPS);
    }
    return (speedRPM/getRatio())/radPsToRPM;
  }
private:
	double radPsToRPM = SEC_PER_MIN / (2*PI);
};

irqISR(irq1,isr1);
MotorHandler fl_wheel(W1_PWM,W1_DIR,W1_IRQ,W1_IRQB,&irq1);

irqISR(irq2,isr2);
MotorHandler rl_wheel(W2_PWM,W2_DIR,W2_IRQ,W2_IRQB,&irq2);

irqISR(irq3,isr3);
MotorHandler rr_wheel(W3_PWM,W3_DIR,W3_IRQ,W3_IRQB,&irq3);

irqISR(irq4,isr4);
MotorHandler fr_wheel(W4_PWM,W4_DIR,W4_IRQ,W4_IRQB,&irq4);

void setDesiredVelocities(MotorHandler* fl,MotorHandler* rl,MotorHandler* rr,MotorHandler* fr, float flCMD, float rlCMD, float rrCMD, float frCMD){
  fl->setDesiredRADPS(flCMD);
  rl->setDesiredRADPS(rlCMD);
  rr->setDesiredRADPS(rrCMD);
  fr->setDesiredRADPS(frCMD);
}
//should be the same as sendDesiredVelocities()
size_t sendEncoderVelocities(MotorHandler* fl,MotorHandler* rl,MotorHandler* rr,MotorHandler* fr){
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

size_t sendDesiredVelocities(MotorHandler* fl,MotorHandler* rl,MotorHandler* rr,MotorHandler* fr){
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

bool PIDEnable(MotorHandler* fl,MotorHandler* rl,MotorHandler* rr,MotorHandler* fr,float kc,float taui,float taud,unsigned int interval) {
	return fl->PIDEnable(kc,taui,taud,interval) &&
			rl->PIDEnable(kc,taui,taud,interval) &&
			rr->PIDEnable(kc,taui,taud,interval) &&
			fr->PIDEnable(kc,taui,taud,interval);
}

void regulatePIDs(MotorHandler* fl,MotorHandler* rl,MotorHandler* rr,MotorHandler* fr){
  return fl->PIDRegulate() && rl->PIDRegulate() && rr->PIDRegulate() && fr->PIDRegulate();
}

void setup() {
  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
	TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz

  Serial.begin(115200);

  PIDEnable(&fl_wheel,&rl_wheel,&rr_wheel,&fr_wheel,0.31,0.01,0,10);
}
bool firstTime = true;
unsigned long lastChange = millis();

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data == "e"){
      sendEncoderVelocities(&fl_wheel,&rl_wheel,&rr_wheel,&fr_wheel);
    } else if (data.startsWith("v:")) { //format: v:<fl>,<fr>,<rl>,<rr>\n`
      data.remove(0,2); //delete v:
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',',firstComma+1);
      int thirdComma = data.indexOf(',',secondComma+1);

      double flCMD = data.substring(0,firstComma).toDouble();
      double frCMD = data.substring(firstComma+1,secondComma).toDouble();
      double rlCMD = data.substring(secondComma+1,thirdComma).toDouble();
      double rrCMD = data.substring(thirdComma+1,data.length()-1).toDouble();

      setDesiredVelocities(&fl_wheel,&rl_wheel,&rr_wheel,&fr_wheel,flCMD,rlCMD,rrCMD,frCMD);
      sendDesiredVelocities(&fl_wheel,&rl_wheel,&rr_wheel,&fr_wheel);
    }
  }

  if (millis()-lastChange >= SAMPLETIME || firstTime){ 
    lastChange = millis();
    regulatePIDs(&fl_wheel,&rl_wheel,&rr_wheel,&fr_wheel);
    firstTime = false;
  }
}