

import math


def pos_to_dist(lat1, lon1, alt1, lat2, lon2, alt2):
        """
        Calcule la distance nord (x), est (y) et verticale (z) en mètres entre deux positions GPS.
        Utilise une approximation locale (petites distances).
        """
        R = 6371000  # Rayon de la Terre en mètres
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        avg_lat = math.radians((lat1 + lat2) / 2)

        dx = d_lat * R  # distance nord (m)
        dy = d_lon * R * math.cos(avg_lat)  # distance est (m)
        dz = alt2 - alt1  # distance verticale (m)

        return dx, dy, dz

lat1, lon1, alt1 = 48.62977063026934, 7.789145334470899, 150 #Follower position
lat2, lon2, alt2 = 48.63083780302854, 7.789510114856793, 155 #Master position

dx, dy, dz = pos_to_dist(lat1, lon1, alt1, lat2, lon2, alt2)
print(f"Distance nord (x): {dx:.2f} m, Distance est (y): {dy:.2f} m, Distance verticale (z): {dz:.2f} m")