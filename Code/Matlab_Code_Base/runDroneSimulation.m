function runDroneSimulation()
    PORT = "COM7";
    ALPHA = 0.98;
    FMT_LEN = 37; % Taille exacte du struct Pkt (35 octets de données + 2 octets de padding C++)
    
    s = serialport(PORT, 921600);
    cleanupObj = onCleanup(@() clear('s'));
    pause(1.5); flush(s);
    
    % Modèle 3D du drone (Box)
    L = 2; W = 1; H = 0.3;
    V = [-L -W -H;  L -W -H;  L  W -H; -L  W -H;
         -L -W  H;  L -W  H;  L  W  H; -L  W  H]/2;
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    C = [0.6 0.6 0.6; 0.2 0.5 1; 0.8 0.8 0.8; 0.8 0.8 0.8; 0.8 0.8 0.8; 1 0.3 0.3];
    
    % Fenêtre graphique de simulation
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
        axis(ax, [-2 2 -2 2 -2 2]); % Limites de -2 à 2 sur les axes
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
        if n > 0
            buf = [buf, read(s, n, "uint8")]; 
        end
        
        while numel(buf) >= 4 + FMT_LEN
            k = find(buf(1:end-1) == 170 & buf(2:end) == 85, 1); 
            if isempty(k)
                buf = buf(end); 
                break; 
            end
            buf = buf(k:end);
            if numel(buf) < 4 + FMT_LEN
                break; 
            end
            
            len = double(buf(3));       
            if len ~= FMT_LEN
                buf = buf(3:end); 
                continue; 
            end
            
            pl = buf(4:3+len);
            cs = uint8(0); 
            for b = pl
                cs = bitxor(cs, b); 
            end
            
            if cs ~= buf(4+len)
                buf = buf(3:end); 
                continue; 
            end
            
            buf = buf(5+len:end);
            
            % --- Décodage précis basé sur les offsets C++ (37 octets) ---
            droneId    = pl(1);
            t          = double(typecast(pl(2:5), 'uint32')) / 1000.0;
            d          = double(typecast(pl(6:9), 'single'));
            
            % GPS (Index MATLAB de 10 à 23)
            lat        = double(typecast(pl(10:13), 'int32')) / 1e7;
            lon        = double(typecast(pl(14:17), 'int32')) / 1e7;
            alt        = double(typecast(pl(18:21), 'int32')) / 1000.0;
            fix_type   = pl(22);
            satellites = pl(23);
            
            % (Octets 24 et 25 = Padding C++ ignorés)
            
            % Attitude (Index MATLAB de 26 à 37)
            roll       = double(typecast(pl(26:29), 'single'));
            pitch      = double(typecast(pl(30:33), 'single'));
            yaw        = double(typecast(pl(34:37), 'single'));
            
            if droneId < 1 || droneId > 3
                continue; 
            end
            
            % Correction d'orientation pour correspondre au drone réel (inversion avant/arrière et gauche/droite)
            roll_corr  = roll;
            pitch_corr = -pitch;
            yaw_corr   = yaw;
            
            % Mise à jour de l'état du drone
            droneState(droneId).roll  = roll_corr;
            droneState(droneId).pitch = -pitch_corr;
            droneState(droneId).yaw   = yaw_corr;
            
            % Matrice de rotation 3D corrigée
            Rx = [1 0 0; 0 cos(roll_corr) -sin(roll_corr); 0 sin(roll_corr) cos(roll_corr)];
            Ry = [cos(pitch_corr) 0 sin(pitch_corr); 0 1 0; -sin(pitch_corr) 0 cos(pitch_corr)];
            Rz = [cos(yaw_corr) -sin(yaw_corr) 0; sin(yaw_corr) cos(yaw_corr) 0; 0 0 1];
            R  = Rz * Ry * Rx;
            
            % Mise à jour du rendu 3D
            h(droneId).Vertices = (R * V')';
            
            % Mise à jour du titre d'affichage
            ttl(droneId).String = {sprintf('Drone %d', droneId), ...
                sprintf('R: %.1f° | P: %.1f° | Y: %.1f°', rad2deg(roll), rad2deg(pitch), rad2deg(yaw)), ...
                sprintf('Dist: %.2fm | Sats: %d', d, satellites)};
        end
        
        drawnow limitrate;
    end
end

function resetDrones()
    global droneState;
    for i = 1:3
        droneState(i).roll  = 0;
        droneState(i).pitch = 0;
        droneState(i).yaw   = 0;
        droneState(i).tPrev = [];
    end
end