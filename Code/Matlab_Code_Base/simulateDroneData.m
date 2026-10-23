function [t, d, z, x_pos, y_pos, lat, lon, alt, satellites, roll, pitch, yaw] = simulateDroneData(simTime, droneId)
    % La fonction reçoit bien simTime et droneId en paramètres d'entrée
    t = simTime;
    d = 1.5 + 0.5 * sin(simTime + droneId);
    z = 5.0 + droneId + 1.0 * cos(simTime * 0.5);
    x_pos = d * cos(droneId * 2 * pi / 3);
    y_pos = d * sin(droneId * 2 * pi / 3);
    
    lat = 48.8566 + droneId * 0.0001;
    lon = 2.3522 + droneId * 0.0001;
    alt = z * 1000; 
    satellites = 12;
    
    roll  = 0.2 * sin(simTime + droneId);
    pitch = 0.2 * cos(simTime);
    yaw   = 0.1 * simTime;
end