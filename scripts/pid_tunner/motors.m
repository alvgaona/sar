function pid_motors_tunner()
    arduinoObj = [];
    testDuration = 3.0;
    
    fig = uifigure('Name', 'Arduino PID Tuner', 'Position', [100, 100, 1000, 650]);
    fig.CloseRequestFcn = @(src, event) onClose(src);
    
    pnl = uipanel(fig, 'Position', [20, 20, 250, 610], 'Title', 'PID Controls');
    
    uilabel(pnl, 'Position', [20, 560, 200, 22], 'Text', 'Port Name (e.g., /dev/ttyUSB0):');
    portEdit = uieditfield(pnl, 'text', 'Position', [20, 535, 120, 22], 'Value', '/dev/ttyUSB0');
    btnConnect = uibutton(pnl, 'push', 'Position', [150, 535, 80, 22], 'Text', 'Connect', 'ButtonPushedFcn', @(btn,event) connectArduino());
    statusLabel = uilabel(pnl, 'Position', [20, 510, 200, 22], 'Text', 'Status: Disconnected', 'FontColor', 'r');
    
    uilabel(pnl, 'Position', [20, 460, 100, 22], 'Text', 'Kp (Proportional):');
    kpEdit = uieditfield(pnl, 'numeric', 'Position', [130, 460, 100, 22], 'Value', 15.0);
    
    uilabel(pnl, 'Position', [20, 420, 100, 22], 'Text', 'Ki (Integral):');
    kiEdit = uieditfield(pnl, 'numeric', 'Position', [130, 420, 100, 22], 'Value', 2.0);
    
    uilabel(pnl, 'Position', [20, 380, 100, 22], 'Text', 'Kd (Derivative):');
    kdEdit = uieditfield(pnl, 'numeric', 'Position', [130, 380, 100, 22], 'Value', 0.05);
    
    uilabel(pnl, 'Position', [20, 320, 100, 22], 'Text', 'Setpoint (rad/s):', 'FontWeight', 'bold');
    spEdit = uieditfield(pnl, 'numeric', 'Position', [130, 320, 100, 22], 'Value', 3.14);
    
    btnRun = uibutton(pnl, 'push', 'Position', [20, 250, 210, 50], 'Text', 'RUN 3s STEP TEST', ...
        'FontWeight', 'bold', 'BackgroundColor', [0.2 0.8 0.2], 'Enable', 'off', ...
        'ButtonPushedFcn', @(btn,event) runTest());
        
    uilabel(pnl, 'Position', [20, 210, 210, 40], 'Text', 'Sends PID -> Applies Step -> Plots 3s -> Stops Motors', 'WordWrap', 'on', 'FontSize', 10);

    btnStop = uibutton(pnl, 'push', 'Position', [20, 150, 210, 40], 'Text', 'EMERGENCY STOP', ...
        'FontWeight', 'bold', 'BackgroundColor', [0.8 0.2 0.2], 'Enable', 'off', ...
        'ButtonPushedFcn', @(btn,event) stopMotors());

    plotContainer = uipanel(fig, 'Position', [280, 20, 700, 610], 'BorderType', 'none');
    axLayout = uigridlayout(plotContainer, [2, 2]);
    axLayout.RowHeight = {'1x', '1x'};
    axLayout.ColumnWidth = {'1x', '1x'};
    
    axesList = gobjects(4,1);
    linesMeas = cell(4,1);
    linesSP = cell(4,1);
    titles = ["Front Left (FL)", "Front Right (FR)", "Rear Left (RL)", "Rear Right (RR)"];
    
    for i = 1:4
        axesList(i) = uiaxes(axLayout);
        title(axesList(i), titles(i));
        xlabel(axesList(i), 'Time (s)');
        ylabel(axesList(i), 'rad/s');
        grid(axesList(i), 'on');
        hold(axesList(i), 'on');
        
        
        linesMeas{i} = animatedline(axesList(i), 'Color', 'b', 'LineWidth', 0.1, 'DisplayName', 'Measured');
        linesSP{i} = animatedline(axesList(i), 'Color', 'r', 'LineStyle', '--', 'LineWidth', 0.1, 'DisplayName', 'Setpoint');
        legend(axesList(i), 'Location', 'southeast');
    end


    function connectArduino()
        try
            if ~isempty(arduinoObj)
                clear arduinoObj;
            end
            port = portEdit.Value;
            statusLabel.Text = 'Connecting...';
            statusLabel.FontColor = '#D9A400';
            drawnow;
            
            arduinoObj = serialport(port, 115200);
            configureTerminator(arduinoObj, "LF");
            flush(arduinoObj);
            pause(2);
            
            statusLabel.Text = 'Status: CONNECTED';
            statusLabel.FontColor = '#00AA00';
            btnRun.Enable = 'on';
            btnStop.Enable = 'on';
        catch ME
            statusLabel.Text = 'Status: FAILED (Check Port)';
            statusLabel.FontColor = 'r';
            uialert(fig, ME.message, 'Connection Error');
        end
    end

    function runTest()
        if isempty(arduinoObj)
            return; 
        end
        
        btnRun.Enable = 'off';
        btnRun.Text = 'TESTING...';
        drawnow;
        
        for j = 1:4
            clearpoints(linesMeas{j});
            clearpoints(linesSP{j});
            axesList(j).XLim = [0, testDuration];
            axesList(j).YLim = [-1, spEdit.Value * 1.5];
        end
        
        flush(arduinoObj);
        pidCmd = sprintf("k:%.3f,%.3f,%.3f", kpEdit.Value, kiEdit.Value, kdEdit.Value);
        writeline(arduinoObj, pidCmd);
        pause(0.1);
        
        sp = spEdit.Value;
        spCmd = sprintf("v:%.3f,%.3f,%.3f,%.3f", sp, sp, sp, sp);
        writeline(arduinoObj, spCmd);
        
        startTime = tic;
        currentTime = 0;
        
        while currentTime < testDuration
            writeline(arduinoObj, "e");
            try
                dataStr = readline(arduinoObj);
            catch
                continue;
            end
            
            if startsWith(dataStr, "e:")
                dataStr = extractAfter(dataStr, "e:");
                vels = str2double(split(dataStr, ","));
                
                if length(vels) == 4
                    currentTime = toc(startTime);
                    
                    for j = 1:4
                        addpoints(linesMeas{j}, currentTime, vels(j));
                        addpoints(linesSP{j}, currentTime, sp);
                    end
                    drawnow limitrate;
                end
            end
        end
        
        stopMotors();
        
        btnRun.Text = 'RUN 3s STEP TEST';
        btnRun.Enable = 'on';
    end

    function stopMotors()
        if ~isempty(arduinoObj)
            writeline(arduinoObj, "s");
            writeline(arduinoObj, "v:0,0,0,0");
        end
    end

    function onClose(src)
        stopMotors();
        if ~isempty(arduinoObj)
            clear arduinoObj;
        end
        delete(src);
    end
end