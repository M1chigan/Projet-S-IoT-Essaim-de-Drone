function runDroneSimulation()
    PORT    = "COM7";
    FMT_LEN = 41;                 % payload non packed (struct GPS avec padding)

    s = serialport(PORT, 921600);
    cleanupObj = onCleanup(@() clear('s')); 
    pause(1.5); flush(s);

    L = 2; W = 1; H = 0.3;
    V = [-L -W -H;  L -W -H;  L  W -H; -L  W -H;
         -L -W  H;  L -W  H;  L  W  H; -L  W  H]/2;
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    C = [0.6 0.6 0.6; 0.2 0.5 1; 0.8 0.8 0.8; 0.8 0.8 0.8; 0.8 0.8 0.8; 1 0.3 0.3];

    droneColors = [1 0.3 0.3; 0.3 1 0.3; 0.3 0.3 1];

    % État partagé (utilisé aussi par resetDrones, fonction imbriquée)
    buf        = uint8([]);
    lat0       = NaN;
    lon0       = NaN;
    droneState = struct('roll', {0,0,0}, 'pitch', {0,0,0}, 'yaw', {0,0,0}, 'tPrev', {[],[],[]});

    %-------------------------------------------------------
    %                  Fenêtre principale
    %-------------------------------------------------------
    fig = uifigure('Name', 'Modélisation de la flotte de drone', ...
        'Position', [100, 100, 1200, 600], 'Color', [0.1, 0.1, 0.1]);
    layout = uigridlayout(fig, [1, 1]);
    layout.Padding = [0 0 0 0];

    tabs = uitabgroup(layout);
    tab1 = uitab(tabs, 'Title', 'Modélisation 3D');
    tab2 = uitab(tabs, 'Title', 'Modélisation de formation');
    tab3 = uitab(tabs, 'Title', 'Valeurs numériques');

    %-------------------------------------------------------
    %                      Onglet 1
    %-------------------------------------------------------
    mainGrid = uigridlayout(tab1, [1, 2]);
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

    axAxes = [uiaxes(vizGrid), uiaxes(vizGrid), uiaxes(vizGrid)];
    h   = gobjects(1, 3);
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

        h(i)   = patch(ax, 'Vertices', V, 'Faces', F, 'FaceVertexCData', C, 'FaceColor', 'flat');
        ttl(i) = title(ax, {sprintf('Drone %d', i), 'Waiting for data...'}, 'Color', 'white');
    end

    %-------------------------------------------------------
    %                      Onglet 2
    %-------------------------------------------------------
    mainGrid_2 = uigridlayout(tab2, [1, 2]);
    mainGrid_2.ColumnWidth = {200, '1x'};
    mainGrid_2.BackgroundColor = [0.15, 0.15, 0.15];

    controlPanel_2 = uipanel(mainGrid_2, 'Title', 'Controls', 'BackgroundColor', [0.2, 0.2, 0.2], 'ForegroundColor', 'white');
    controlGrid_2 = uigridlayout(controlPanel_2, [4, 1]);
    controlGrid_2.RowHeight = {40, 40, 40, '1x'};
    btnReset2 = uibutton(controlGrid_2, 'Text', 'Reset formation', ...
        'ButtonPushedFcn', @(~,~) resetDrones());
    btnReset2.BackgroundColor = [0.8, 0.3, 0.3];
    btnReset2.FontColor = 'white';

    vizGrid_2 = uigridlayout(mainGrid_2, [1, 2]);
    vizGrid_2.Layout.Column = 2;
    vizGrid_2.BackgroundColor = [0.1, 0.1, 0.1];

    axP = uiaxes(vizGrid_2);          % profil
    hold(axP, 'on'); grid(axP, 'on');
    axis(axP, 'equal');
    xlim(axP, [0 60]); ylim(axP, [0 50]);
    view(axP, 2);
    axP.Color = 'k'; axP.XColor = 'w'; axP.YColor = 'w';
    xlabel(axP, 'Distance à la base (m)'); ylabel(axP, 'Altitude relative z (m)');
    title(axP, 'Formation (Vue de profil)', 'Color', 'white');

    axD = uiaxes(vizGrid_2);          % dessus
    hold(axD, 'on'); grid(axD, 'on');
    axis(axD, 'equal');
    xlim(axD, [-60 60]); ylim(axD, [-60 60]);
    view(axD, 2);
    axD.Color = 'k'; axD.XColor = 'w'; axD.YColor = 'w';
    xlabel(axD, 'Est (m)'); ylabel(axD, 'Nord (m)');
    title(axD, 'Formation (Vue de dessus)', 'Color', 'white');

    h_profil = gobjects(1, 3);
    h_dessus = gobjects(1, 3);
    for i = 1:3
        h_profil(i) = plot(axP, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', droneColors(i,:), 'MarkerEdgeColor', 'w');
        h_dessus(i) = plot(axD, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', droneColors(i,:), 'MarkerEdgeColor', 'w');
    end

    %-------------------------------------------------------
    %                Onglet 3 : Valeurs numériques
    %-------------------------------------------------------
    mainGrid_3 = uigridlayout(tab3, [1, 3]);
    mainGrid_3.BackgroundColor = [0.15, 0.15, 0.15];

    tblData = gobjects(1, 3);
    paramNames = {'Roll (°)'; 'Pitch (°)'; 'Yaw (°)'; 'Distance (m)'; 'Altitude (m)'; ...
                  'Latitude'; 'Longitude'; 'Satellites'; 'Temps (s)'};

    for i = 1:3
        pnl = uipanel(mainGrid_3, 'Title', sprintf('Drone %d', i), ...
            'BackgroundColor', [0.2, 0.2, 0.2], 'ForegroundColor', 'white', 'FontSize', 14);
        pnlLayout = uigridlayout(pnl, [1, 1]);
        pnlLayout.Padding = [5 5 5 5];
        tblData(i) = uitable(pnlLayout, ...
            'ColumnName', {'Paramètre', 'Valeur'}, ...
            'ColumnWidth', {120, '1x'}, ...
            'RowName', [], ...
            'Data', [paramNames, repmat({'--'}, 9, 1)], ...
            'BackgroundColor', [0.15 0.15 0.15; 0.2 0.2 0.2], ...
            'ForegroundColor', 'white');
    end

    %-------------------------------------------------------
    %                  Boucle principale
    %-------------------------------------------------------
    while isvalid(fig)

        % 1) Lecture série
        n = s.NumBytesAvailable;
        if n > 0
            data = read(s, n, "uint8");
            buf  = [buf, uint8(data(:).')];
        end

        % 2) Décodage des trames AA 55 | len | payload | XOR
        while numel(buf) >= 4 + FMT_LEN
            k = find(buf(1:end-1) == 170 & buf(2:end) == 85, 1);
            if isempty(k), buf = buf(end); break; end
            buf = buf(k:end);
            if numel(buf) < 4 + FMT_LEN, break; end

            len = double(buf(3));
            if len ~= FMT_LEN, buf = buf(3:end); continue; end

            pl = uint8(buf(4:3+len));
            cs = uint8(0);
            for b = pl, cs = bitxor(cs, b); end
            if cs ~= buf(4+len), buf = buf(3:end); continue; end
            buf = buf(5+len:end);

            droneId = double(pl(1));
            if droneId < 1 || droneId > 3, continue; end

            t          = double(typecast(pl(2:5),   'uint32')) / 1000.0;
            d          = double(typecast(pl(6:9),   'single'));
            z          = double(typecast(pl(10:13), 'single'));
            lat        = double(typecast(pl(14:17), 'int32')) / 1e7;
            lon        = double(typecast(pl(18:21), 'int32')) / 1e7;
            alt        = double(typecast(pl(22:25), 'int32')) / 1000.0;
            satellites = double(pl(27));
            % pl(28:29) = padding
            roll       = double(typecast(pl(30:33), 'single'));
            pitch      = double(typecast(pl(34:37), 'single'));
            yaw        = double(typecast(pl(38:41), 'single'));

            roll_corr  = deg2rad(roll);
            pitch_corr = -deg2rad(pitch);
            yaw_corr   = deg2rad(yaw);

            droneState(droneId).roll  = roll_corr;
            droneState(droneId).pitch = pitch_corr;
            droneState(droneId).yaw   = yaw_corr;

            Rx = [1 0 0; 0 cos(roll_corr) -sin(roll_corr); 0 sin(roll_corr) cos(roll_corr)];
            Ry = [cos(pitch_corr) 0 sin(pitch_corr); 0 1 0; -sin(pitch_corr) 0 cos(pitch_corr)];
            Rz = [cos(yaw_corr) -sin(yaw_corr) 0; sin(yaw_corr) cos(yaw_corr) 0; 0 0 1];
            R  = Rz * Ry * Rx;
            h(droneId).Vertices = (R * V')';

            if isnan(lat0), lat0 = lat; lon0 = lon; end
            x = (lon - lon0) * 111320 * cosd(lat0);   % est (m)
            y = (lat - lat0) * 111320;                % nord (m)

           ttl(droneId).String = {sprintf('Drone %d', droneId), ...
                sprintf('R: %.1f° | P: %.1f° | Y: %.1f°', roll, pitch, yaw), ...
                sprintf('Dist: %.2fm | Sats: %d', d, satellites)};

            set(h_profil(droneId), 'XData', d, 'YData', z);
            set(h_dessus(droneId), 'XData', x, 'YData', y);

            newValues = { ...
                sprintf('%.2f', roll); sprintf('%.2f', pitch); sprintf('%.2f', yaw); ...
                sprintf('%.2f', d); sprintf('%.2f', alt); ...
                sprintf('%.7f', lat); sprintf('%.7f', lon); ...
                sprintf('%d', satellites); sprintf('%.3f', t)};
            tblData(droneId).Data(:, 2) = newValues;
        end

        % 3) Rafraîchissement de l'interface
        drawnow limitrate;
    end

    %-------------------------------------------------------
    %                Fonction imbriquée
    %-------------------------------------------------------
    function resetDrones()
        for ii = 1:3
            droneState(ii).roll  = 0;
            droneState(ii).pitch = 0;
            droneState(ii).yaw   = 0;
            droneState(ii).tPrev = [];
        end
        lat0 = NaN;           % la vue de dessus se recale sur la prochaine trame
        lon0 = NaN;
        buf  = uint8([]);
    end

end