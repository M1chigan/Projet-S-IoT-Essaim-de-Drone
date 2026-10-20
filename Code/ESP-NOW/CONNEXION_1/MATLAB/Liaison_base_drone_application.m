% Main entry point script
clear; close all;

% Create the main window (Menu)
mainFig = uifigure('Name', 'Main Menu', 'Position', [500, 400, 400, 250], 'Color', [0.15, 0.15, 0.15]);

mainLayout = uigridlayout(mainFig, [3, 1]);
mainLayout.RowHeight = {'1x', 50, '1x'};
mainLayout.BackgroundColor = [0.15, 0.15, 0.15];

titleLabel = uilabel(mainLayout, 'Text', 'Drone Monitoring System', ...
    'FontSize', 18, 'FontColor', 'white', 'HorizontalAlignment', 'center');
titleLabel.Layout.Row = 1;

launchBtn = uibutton(mainLayout, 'Text', 'Start 3D Simulation', ...
    'ButtonPushedFcn', @(~,~) startSimulation(mainFig));
launchBtn.Layout.Row = 2;
launchBtn.BackgroundColor = [0.2, 0.6, 0.3];
launchBtn.FontColor = 'white';
launchBtn.FontSize = 14;

% --- Callback function to launch simulation ---
function startSimulation(parentFig)
    % Close the main menu window
    close(parentFig);
    
    % Run the 3D simulation
    runDroneSimulation();
end

% --- 3D Simulation Function ---
function runDroneSimulation()
    PORT = "COM7";
    DRONE = 1;
    ALPHA = 0.98;
    FMT_LEN = 29;

    s = serialport(PORT, 921600);
    cleanupObj = onCleanup(@() clear('s'));
    pause(1.5); flush(s);

    % 3D Model Box
    L = 2; W = 1; H = 0.3;
    V = [-L -W -H;  L -W -H;  L  W -H; -L  W -H;
         -L -W  H;  L -W  H;  L  W  H; -L  W  H]/2;
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    C = [0.6 0.6 0.6; 0.2 0.5 1; 0.8 0.8 0.8; 0.8 0.8 0.8; 0.8 0.8 0.8; 1 0.3 0.3];

    % Simulation GUI Figure
    simFig = uifigure('Name', '3D Drone Fleet Simulation', 'Position', [100, 100, 1200, 550], 'Color', [0.1, 0.1, 0.1]);
    
    mainGrid = uigridlayout(simFig, [1, 2]);
    mainGrid.ColumnWidth = {200, '1x'};
    mainGrid.BackgroundColor = [0.15, 0.15, 0.15];

    controlPanel = uipanel(mainGrid, 'Title', 'Controls', 'BackgroundColor', [0.2, 0.2, 0.2], 'ForegroundColor', 'white');
    controlGrid = uigridlayout(controlPanel, [4, 1]);
    controlGrid.RowHeight = {40, 40, 40, '1x'};

    btnReset = uibutton(controlGrid, 'Text', 'Reset Angles', ...
        'ButtonPushedFcn', @(~,~) resetDrones());
    btnReset.BackgroundColor = [0.8, 0.3, 0.3];
    btnReset.FontColor = 'white';

    vizGrid = uigridlayout(mainGrid, [1, 3]);
    vizGrid.BackgroundColor = [0.1, 0.1, 0.1];

    ax1 = uiaxes(vizGrid); ax2 = uiaxes(vizGrid); ax3 = uiaxes(vizGrid);
    axAxes = [ax1, ax2, ax3];
    h = gobjects(1, 3);
    ttl = gobjects(1, 3);

    for i = 1:3
        ax = axAxes(i);
        hold(ax, 'on');
        grid(ax, 'on');
        axis(ax, 'equal');
        axis(ax, [-2 2 -2 2 -2 2]);
        view(ax, 135, 25);
        ax.Color = 'k';
        ax.XColor = 'w'; ax.YColor = 'w'; ax.ZColor = 'w';
        xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
        
        h(i) = patch(ax, 'Vertices', V, 'Faces', F, 'FaceVertexCData', C, 'FaceColor', 'flat');
        ttl(i) = title(ax, {sprintf('Drone %d', i), 'Waiting for data...'}, 'Color', 'white');
    end

    global droneState;
    resetDrones();

    buf = uint8([]);

    while ishandle(simFig)
        n = s.NumBytesAvailable;
        if n > 0, buf = [buf, read(s, n, "uint8")]; end
        
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
            
            td = pl(1);
            if td < 1 || td > 3, continue, end
            
            t   = double(typecast(pl(2:5), 'uint32'))/1000;
            imu = double(typecast(pl(6:29), 'single'));
            a = imu(1:3); g = imu(4:6);
            
            if isempty(droneState(td).tPrev)
                droneState(td).tPrev = t; 
                continue; 
            end
            dt = t - droneState(td).tPrev; 
            droneState(td).tPrev = t;
            
            if dt <= 0 || dt > 0.5, continue, end
            
            r = droneState(td).roll;
            p = droneState(td).pitch;
            y = droneState(td).yaw;
            
            gx = g(1); gy = g(2); gz = g(3);
            rollR  = gx + sin(r)*tan(p)*gy + cos(r)*tan(p)*gz;
            pitchR = cos(r)*gy - sin(r)*gz;
            yawR   = (sin(r)*gy + cos(r)*gz) / cos(p);
            
            r = r + rollR*dt;
            p = p + pitchR*dt;
            y = y + yawR*dt;
            
            rollAcc  = atan2(a(2), a(3));
            pitchAcc = atan2(-a(1), sqrt(a(2)^2 + a(3)^2));
            
            r = ALPHA*r + (1-ALPHA)*rollAcc;
            p = ALPHA*p + (1-ALPHA)*pitchAcc;
            p = max(min(p, 1.5), -1.5);
            
            droneState(td).roll = r;
            droneState(td).pitch = p;
            droneState(td).yaw = y;
            
            Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
            Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
            Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
            R  = Rz*Ry*Rx;
            
            h(td).Vertices = (R*V')';
            ttl(td).String = {sprintf('Drone %d', td), sprintf('Roll %.1f°   Pitch %.1f°   Yaw %.1f°', rad2deg(r), rad2deg(p), rad2deg(y))};
        end
        
        drawnow limitrate;
    end
end

function resetDrones()
    global droneState;
    for i = 1:3
        droneState(i).roll = 0;
        droneState(i).pitch = 0;
        droneState(i).yaw = 0;
        droneState(i).tPrev = [];
    end
end