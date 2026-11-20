function runObserver()
    PORT = "COM7";
    BAUD = 921600;
    FMT_LEN = 37; % Taille exacte calculée depuis votre struct packed
    
    % Open serial port
    s = serialport(PORT, BAUD);
    cleanupObj = onCleanup(@() clear('s'));
    flush(s);
    
    % Create UI window for UART monitoring
    monitorFig = uifigure('Name', 'UART Terminal & Data Observer', 'Position', [150, 150, 850, 550], 'Color', [0.15, 0.15, 0.15]);
   
    layout = uigridlayout(monitorFig, [3, 1]);
    layout.RowHeight = {'1x', 60};
    layout.BackgroundColor = [0.15, 0.15, 0.15];
    
    % Text area acting as a terminal log
    logArea = uitextarea(layout, 'Editable', 'off', 'BackgroundColor', 'black', 'FontColor', 'green', 'FontName', 'FixedWidth', 'FontSize', 12);
    logArea.Layout.Row = 1;
    
    % Control panel at the bottom (3 colonnes pour 3 boutons)
    bottomGrid = uigridlayout(layout, [1, 3]);
    bottomGrid.ColumnWidth = {200, 120, 120};
    bottomGrid.BackgroundColor = [0.15, 0.15, 0.15];
    bottomGrid.Layout.Row = 2;
    
    % State button to switch mode (Colonne 1)
    modeBtn = uibutton(bottomGrid, 'state', ...
        'Text', 'Mode: Hexadecimal', ...
        'ValueChangedFcn', @(btn, ~) updateModeText(btn), ...
        'BackgroundColor', [0.2, 0.4, 0.7], ...
        'FontColor', 'white', ...
        'FontSize', 12);
    modeBtn.Layout.Column = 1;
    
    % Clear button (Colonne 2)
    btnClear = uibutton(bottomGrid, 'Text', 'Clear Terminal', ...
        'ButtonPushedFcn', @(~,~) set(logArea, 'Value', {''}));
    btnClear.Layout.Column = 2;
    btnClear.BackgroundColor = [0.7, 0.3, 0.3];
    btnClear.FontColor = 'white';
    
    % Back button (Colonne 3)
    BackButton = uibutton(bottomGrid, 'push', ...
        'Text', 'Retour', ...
        'ButtonPushedFcn', @(~,~) BackButtonFunc(monitorFig), ...
        'BackgroundColor', [0.2, 0.4, 0.5], ...
        'FontColor', 'white', ...
        'FontSize', 12);
    BackButton.Layout.Column = 3;
    
    buf = uint8([]);
    
    % Read loop
    while ishandle(monitorFig)
        n = s.NumBytesAvailable;
        if n > 0
            newBytes = read(s, n, "uint8");
            buf = [buf, newBytes];
            
            currentVal = logArea.Value;
            if isempty(currentVal), currentVal = {''}; end
            
            if modeBtn.Value 
                currentMode = 'Formatted Data';
            else
                currentMode = 'Hexadecimal';
            end
            
            if strcmp(currentMode, 'Hexadecimal')
                % --- Raw Hexadecimal Mode ---
                hexStr = upper(string(dec2hex(newBytes, 2)));
                hexLine = strjoin(hexStr, ' ');
                
                if numel(currentVal) > 150
                    currentVal = currentVal(end-120:end);
                end
                logArea.Value = [currentVal; char(hexLine)];
                
            else
                % --- Formatted Data Mode (Frame Parsing) ---
                while numel(buf) >= 4 + FMT_LEN
                    k = find(buf(1:end-1) == 170 & buf(2:end) == 85, 1); 
                    if isempty(k), buf = buf(end); break, end
                    buf = buf(k:end);
                    if numel(buf) < 4 + FMT_LEN, break, end
                    
                    len = double(buf(3));       
                    if len ~= FMT_LEN, buf = buf(3:end); continue, end
                    
                    pl = buf(4:3+len);
                    cs = uint8(0); for b = pl, cs = bitxor(cs,b); end
                    if cs ~= buf(4+len), buf = buf(3:end); continue, end
                    
                    buf = buf(5+len:end);
                    
                    % --- Décodage précis basé sur les offsets C++ ---
                    droneId    = pl(1);
                    t          = double(typecast(pl(2:5), 'uint32')) / 1000.0;
                    d          = double(typecast(pl(6:9), 'single'));
                    
                    % GPS (Index MATLAB de 10 à 23)
                    lat        = double(typecast(pl(10:13), 'int32')) / 1e7;
                    lon        = double(typecast(pl(14:17), 'int32')) / 1e7;
                    alt        = double(typecast(pl(18:21), 'int32')) / 1000.0;
                    fix_type   = pl(22);
                    satellites = pl(23);
                    
                    % (Les octets 24 et 25 correspondent aux 2 octets de padding du compilateur -> on les ignore)
                    
                    % Attitude (Index MATLAB de 26 à 37)
                    roll       = double(typecast(pl(26:29), 'single'));
                    pitch      = double(typecast(pl(30:33), 'single'));
                    yaw        = double(typecast(pl(34:37), 'single'));
                    
                    % Ligne de log formatée avec toutes les données
                    lineStr = sprintf('[Drone %d] t:%.3fs | Dist:%.2fm | GPS(Lat:%.5f, Lon:%.5f, Alt:%.1fm, Fix:%d, Sats:%d) | Att(R:%.2f, P:%.2f, Y:%.2f)', ...
                        droneId, t, d, lat, lon, alt, fix_type, satellites, rad2deg(roll), rad2deg(pitch), rad2deg(yaw));
                    
                    if numel(currentVal) > 150
                        currentVal = currentVal(end-120:end);
                    end
                    currentVal = [currentVal; char(lineStr)]; %#ok<AGROW>
                end
                logArea.Value = currentVal;
            end
        end
        drawnow limitrate;
    end
end

function updateModeText(btn)
    if btn.Value
        btn.Text = 'Mode: Formatted Data';
        btn.BackgroundColor = [0.2, 0.6, 0.3];
    else
        btn.Text = 'Mode: Hexadecimal';
        btn.BackgroundColor = [0.2, 0.4, 0.7];
    end
end

function BackButtonFunc(MonitorFig)
close(MonitorFig);
mainMenu();
end