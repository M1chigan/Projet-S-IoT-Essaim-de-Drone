clear; close all;
PORT   = "COM7";          % adapte ton port
DRONE  = 1;               % id du drone à afficher
ALPHA  = 0.98;            % filtre complémentaire : confiance dans le gyro
FMT_LEN = 29;
td = 1;
s = serialport(PORT, 921600);
cleanupObj = onCleanup(@() clear('s'));   % libère le port à la fin
pause(1.5); flush(s);

% ---------- Drone 3D (longueur x largeur x hauteur) ----------
L = 2; W = 1; H = 0.3;
V = [-L -W -H;  L -W -H;  L  W -H; -L  W -H;
     -L -W  H;  L -W  H;  L  W  H; -L  W  H]/2;
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
C = [0.6 0.6 0.6; 0.2 0.5 1; 0.8 0.8 0.8; 0.8 0.8 0.8; 0.8 0.8 0.8; 1 0.3 0.3];

%------------ Figure Drones ---------------
figure = figure('Name','Simulation des drones','Color','k');

%------------ Figure Drone 1 ---------------
ax = gca;
fig1 = subplot(1,3,1,ax);
hold(fig1,'on');
grid(fig1,'on');
axis(fig1,'equal');
axis(fig1,[-2 2 -2 2 -2 2]);
view(fig1,135,25)
xlabel('X (avant)'); 
ylabel('Y'); 
zlabel('Z');
h1 = patch(fig1,'Vertices',V,'Faces',F,'FaceVertexCData',C,'FaceColor','flat');
ttl1 = title(fig1,{'Drone 1','En attente de données...'});

%------------ Figure Drone 2 ---------------
fig2 = subplot(1,3,2);
hold(fig2,'on');
grid(fig2,'on');
axis(fig2,'equal');
axis(fig2,[-2 2 -2 2 -2 2]);
view(fig2,135,25)
xlabel('X (avant)'); 
ylabel('Y'); 
zlabel('Z');
h2 = patch(fig2,'Vertices',V,'Faces',F,'FaceVertexCData',C,'FaceColor','flat');
ttl2 = title(fig2,{'Drone 2','En attente de données...'});

%------------ Figure Drone 3 ---------------
fig3 = subplot(1,3,3);
hold(fig3,'on');
grid(fig3,'on');
axis(fig3,'equal');
axis(fig3,[-2 2 -2 2 -2 2]);
view(fig3,135,25)
xlabel('X (avant)'); 
ylabel('Y'); 
zlabel('Z');
h3 = patch(fig3,'Vertices',V,'Faces',F,'FaceVertexCData',C,'FaceColor','flat');
ttl3 = title(fig3,{'Drone 3','En attente de données...'});

% ---------- État du filtre ----------
roll = 0; pitch = 0; yaw = 0;
tPrev = []; buf = uint8([]);

while ishandle(figure)
    n = s.NumBytesAvailable;
    if n > 0, buf = [buf, read(s, n, "uint8")]; end
    
    %Recherche et vérification Trame d'En Tête
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
        t   = double(typecast(pl(2:5), 'uint32'))/1000;
        imu = double(typecast(pl(6:29), 'single'));
        a = imu(1:3); g = imu(4:6);

        if isempty(tPrev), tPrev = t; continue, end
        dt = t - tPrev; tPrev = t;
        if dt <= 0 || dt > 0.5, continue, end

        % 1) Intégration du gyro (vitesses d'Euler)
        gx = g(1); gy = g(2); gz = g(3);
        rollR  = gx + sin(roll)*tan(pitch)*gy + cos(roll)*tan(pitch)*gz;
        pitchR = cos(roll)*gy - sin(roll)*gz;
        yawR   = (sin(roll)*gy + cos(roll)*gz) / cos(pitch);
        roll  = roll  + rollR*dt;
        pitch = pitch + pitchR*dt;
        yaw   = yaw   + yawR*dt;

        % 2) Correction par l'accéléromètre (roll/pitch uniquement)
        rollAcc  = atan2(a(2), a(3));
        pitchAcc = atan2(-a(1), sqrt(a(2)^2 + a(3)^2));
        roll  = ALPHA*roll  + (1-ALPHA)*rollAcc;
        pitch = ALPHA*pitch + (1-ALPHA)*pitchAcc;
        pitch = max(min(pitch, 1.5), -1.5);   % évite la singularité à ±90°
    end

    % ---------- Affichage ----------
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R  = Rz*Ry*Rx;
    switch td
        case 1
            ttl1.String = {'Drone 1',sprintf('Roll %.1f°   Pitch %.1f°   Yaw %.1f°', rad2deg(roll), rad2deg(pitch), rad2deg(yaw))};
            h1.Vertices = (R*V')';
        case 2
            ttl2.String = {'Drone 2',sprintf('Roll %.1f°   Pitch %.1f°   Yaw %.1f°', rad2deg(roll), rad2deg(pitch), rad2deg(yaw))};
            h2.Vertices = (R*V')';
        case 3 
            ttl3.String = {'Drone 3',sprintf('Roll %.1f°   Pitch %.1f°   Yaw %.1f°', rad2deg(roll), rad2deg(pitch), rad2deg(yaw))};
            h3.Vertices = (R*V')';
    end
    drawnow limitrate;
end