#!/usr/bin/env python3
"""
Parse a JSON log (like `log_2011.json`) produced by `test_correcteurs.py`,
extract positions (`self_position`), position errors (`position_error`) and
velocity commands (`velocity`), save a CSV and plot 2D positions colored by time.

Usage:
    python plot_positions.py --input log_2011.json --out plot.png --csv out.csv

The script is robust to entries where values are strings like
  '48.5824, 7.76407, 180' or large integer lat/lon in 1e7 units.
"""
import json
import math
import os
import re
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np


FLOAT_RE = re.compile(r"[-+]?\d*\.\d+|[-+]?\d+")


def extract_floats(s: str):
    if s is None:
        return []
    found = FLOAT_RE.findall(str(s))
    return [float(x) for x in found]


def normalize_latlon(lat: float, lon: float):
    # Heuristic: if lat/ lon are extremely large (like 1e7 ints), scale down.
    if abs(lat) > 180:
        lat = lat / 1e7
    if abs(lon) > 180:
        lon = lon / 1e7
    return lat, lon


def latlon_to_local_meters(lat, lon, lat0, lon0):
    # approximate conversion: meters per degree
    lat_rad = math.radians(lat)
    lat0_rad = math.radians(lat0)
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos((lat_rad + lat0_rad) / 2.0)
    dx = (lon - lon0) * m_per_deg_lon
    dy = (lat - lat0) * m_per_deg_lat
    return dx, dy


def parse_log(path: str):
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    times = []
    lats = []
    lons = []
    alts = []
    dxs = []
    dys = []
    dzs = []
    vxs = []
    vys = []
    vzs = []

    idx = 0
    for entry in data:
        # many entries may be empty dicts; guard access
        entry_data = entry.get('data') if isinstance(entry, dict) else None
        # default values
        lat = lon = alt = None
        edx = edy = edz = None
        vx = vy = vz = None

        if isinstance(entry_data, dict):
            for k, v in entry_data.items():
                key = str(k).lower()
                if 'self_position' in key or 'self position' in key or 'self_position' == key:
                    vals = extract_floats(v)
                    if len(vals) >= 2:
                        lat, lon = vals[0], vals[1]
                        if len(vals) >= 3:
                            alt = vals[2]
                elif 'position_error' in key or 'position error' in key:
                    vals = extract_floats(v)
                    if len(vals) >= 2:
                        edx = vals[0]
                        edy = vals[1]
                        if len(vals) >= 3:
                            edz = vals[2]
                elif 'velocity' in key:
                    vals = extract_floats(v)
                    if len(vals) >= 2:
                        vx = vals[0]
                        vy = vals[1]
                        if len(vals) >= 3:
                            vz = vals[2]
                elif key.strip() == 'alt':
                    vals = extract_floats(v)
                    if vals:
                        alt = vals[0]

        # If lat/lon are still None, skip
        times.append(entry.get('time', '') if isinstance(entry, dict) else '')
        lats.append(lat)
        lons.append(lon)
        alts.append(alt)
        dxs.append(edx)
        dys.append(edy)
        dzs.append(edz)
        vxs.append(vx)
        vys.append(vy)
        vzs.append(vz)
        idx += 1

    return {
        'time': times,
        'lat': lats,
        'lon': lons,
        'alt': alts,
        'err_x': dxs,
        'err_y': dys,
        'err_z': dzs,
        'vx': vxs,
        'vy': vys,
        'vz': vzs,
    }


def save_csv_grouped(out_csv: str, parsed: dict):
    """Write CSV grouping position, error and velocity when they follow each other.

    Rules:
    - If a `self_position` entry is at index i, and `position_error` is at i+1,
      and `velocity` is at i+2, they are considered the same timestamp and
      written on one line.
    - The code advances the index past consumed entries so groups aren't duplicated.
    - If fields are missing, empty cells are written.
    """
    import csv

    keys = ['index', 'time_pos', 'lat', 'lon', 'alt', 'err_x', 'err_y', 'err_z', 'vx', 'vy', 'vz', 'speed_norm']
    n = len(parsed['time'])
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        i = 0
        while i < n:
            # initialize empty values
            time_pos = parsed['time'][i] if i < n else ''
            lat = parsed['lat'][i] if i < n else None
            lon = parsed['lon'][i] if i < n else None
            alt = parsed['alt'][i] if i < n else None
            err_x = parsed['err_x'][i] if i < n else None
            err_y = parsed['err_y'][i] if i < n else None
            err_z = parsed['err_z'][i] if i < n else None
            vx = parsed['vx'][i] if i < n else None
            vy = parsed['vy'][i] if i < n else None
            vz = parsed['vz'][i] if i < n else None

            consumed = 1

            # If this entry contains a position, check next entries for error and velocity
            if lat is not None and lon is not None:
                # look for error at i+1
                if i + 1 < n and parsed['err_x'][i + 1] is not None or i + 1 < n and parsed['err_y'][i + 1] is not None:
                    err_x = parsed['err_x'][i + 1]
                    err_y = parsed['err_y'][i + 1]
                    err_z = parsed['err_z'][i + 1] if i + 1 < n else err_z
                    consumed = 2
                # look for velocity at i+2 (or i+1 if error wasn't present)
                vel_idx = i + consumed
                if vel_idx < n and (parsed['vx'][vel_idx] is not None or parsed['vy'][vel_idx] is not None):
                    vx = parsed['vx'][vel_idx]
                    vy = parsed['vy'][vel_idx]
                    vz = parsed['vz'][vel_idx] if vel_idx < n else vz
                    consumed = vel_idx - i + 1

            else:
                # No position at i: if this entry is an error and next is velocity, group them
                if (parsed['err_x'][i] is not None or parsed['err_y'][i] is not None):
                    err_x = parsed['err_x'][i]
                    err_y = parsed['err_y'][i]
                    err_z = parsed['err_z'][i]
                    # check next for velocity
                    if i + 1 < n and (parsed['vx'][i + 1] is not None or parsed['vy'][i + 1] is not None):
                        vx = parsed['vx'][i + 1]
                        vy = parsed['vy'][i + 1]
                        vz = parsed['vz'][i + 1]
                        consumed = 2

            # compute speed norm
            try:
                speed_norm = math.hypot(float(vx), float(vy)) if (vx is not None and vy is not None) else ''
            except Exception:
                speed_norm = ''

            row = [i, time_pos, lat, lon, alt, err_x, err_y, err_z, vx, vy, vz, speed_norm]
            writer.writerow(row)

            i += consumed


def plot_positions(parsed: dict, draw_vel: bool = True, goal_lat: Optional[float] = None, goal_lon: Optional[float] = None, plot_speed: bool = False, speed_out: Optional[str] = None, plot_error: bool = False, error_out: Optional[str] = None):
    lat_list = parsed['lat']
    lon_list = parsed['lon']
    vx_list = parsed['vx']
    vy_list = parsed['vy']

    # Filter valid positions
    valid_idx = [i for i, (la, lo) in enumerate(zip(lat_list, lon_list)) if la is not None and lo is not None]
    if not valid_idx:
        raise RuntimeError('No valid positions found in log')

    vel_idx = [i for i, (vx, vy) in enumerate(zip(vx_list, vy_list)) if vx is not None and vy is not None]
    first = valid_idx[0]
    lat0 = float(lat_list[first])
    lon0 = float(lon_list[first])

    xs = []
    ys = []
    vxs_m = []
    vys_m = []
    times = []
    for i in valid_idx:
        lat, lon = normalize_latlon(float(lat_list[i]), float(lon_list[i]))
        dx_m, dy_m = latlon_to_local_meters(lat, lon, lat0, lon0)
        xs.append(dx_m)
        ys.append(dy_m)
    for i in vel_idx:
        # velocities are in m/s already from test_correcteurs
        vxs_m.append(None if vx_list[i] is None else float(vx_list[i]))
        vys_m.append(None if vy_list[i] is None else float(vy_list[i]))
        times.append(i)

    xs = np.array(xs)
    ys = np.array(ys)
    times = np.array(times)

    # compute speed norm (vx, vy)
    speed = np.array([math.hypot(u if u is not None else 0.0, v if v is not None else 0.0) for u, v in zip(vxs_m, vys_m)])
    # compute error norm (err_x, err_y) for the same valid indices
    
    err_x_list = parsed.get('err_x', [])
    err_y_list = parsed.get('err_y', [])

    err_norm = np.array([
        math.hypot(float(err_x_list[i]), float(err_y_list[i]))
        for i in valid_idx
        if err_x_list[i] is not None and err_y_list[i] is not None
    ])

    if err_norm.size == 0:
        print("Warning: No valid position error data found. Skipping error plot.")
        plot_error = False

    plt.figure(figsize=(9, 7))
    sc = plt.scatter(xs, ys, c=times, cmap='viridis', s=20)
    plt.colorbar(sc, label='log index (time order)')
    plt.plot(xs, ys, color='gray', linewidth=0.5, alpha=0.6)
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('2D Trajectory colored by time')

    if draw_vel:
        # draw velocity arrows for subset to avoid clutter
        Qskip = max(1, len(xs) // 40)
        u = []
        v = []
        for vx, vy in zip(vxs_m, vys_m):
            if vx is None or vy is None:
                u.append(0.0)
                v.append(0.0)
            else:
                # Note: the move_velocity call used vx,vy as m/s in body or world frame
                # We'll plot them directly (scale down visually)
                u.append(vx)
                v.append(vy)
        u = np.array(u)
        v = np.array(v)
        plt.quiver(xs[::Qskip], ys[::Qskip], u[::Qskip], v[::Qskip], color='r', width=0.003, scale=5)

    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.4)
    # Plot goal as a big red cross if provided
    if goal_lat is not None and goal_lon is not None:
        # normalize and convert goal to local meters relative to first point
        g_lat, g_lon = normalize_latlon(float(goal_lat), float(goal_lon))
        gx, gy = latlon_to_local_meters(g_lat, g_lon, lat0, lon0)
        # large red cross marker
        plt.scatter([gx], [gy], marker='x', c='red', s=200, linewidths=3, label='Goal')
        plt.plot([gx - 10, gx + 10], [gy, gy], color='red', linewidth=2)
        plt.plot([gx, gx], [gy - 10, gy + 10], color='red', linewidth=2)
        plt.legend()
    if False:
        plt.tight_layout()
        plt.savefig(out_png, dpi=200)
        print('Saved plot to', out_png)
    else:
        plt.show()

    # plot speed norm vs log index if requested
    if plot_speed:
        plt.figure(figsize=(8, 3.5))
        plt.plot(times, speed, marker='o', markersize=3, linestyle='-', color='tab:blue')
        plt.xlabel('log index')
        plt.ylabel('speed norm (m/s)')
        plt.title('Velocity norm (sqrt(vx^2+vy^2)) over log index')
        plt.grid(True, linestyle='--', alpha=0.4)
        if speed_out:
            plt.tight_layout()
            plt.savefig(speed_out, dpi=150)
            print('Saved speed plot to', speed_out)
        else:
            plt.show()

    # plot error norm vs log index if requested
    if plot_error:
        plt.figure(figsize=(8, 3.5))
        plt.plot(times, err_norm, marker='o', markersize=3, linestyle='-', color='tab:orange')
        plt.xlabel('log index')
        plt.ylabel('position error norm (m)')
        plt.title('Position error norm (sqrt(err_x^2+err_y^2)) over log index')
        plt.grid(True, linestyle='--', alpha=0.4)
        if error_out:
            plt.tight_layout()
            plt.savefig(error_out, dpi=150)
            print('Saved error plot to', error_out)
        else:
            plt.show()



def main():
    # Hard-coded configuration (no CLI)
    input_path = os.path.join('Mission3/Analyse_logs', 'log_2011.json')
    out_csv = os.path.join('Mission3/Analyse_logs/csvs', 'extracted_grouped.csv')
    speed_out = os.path.join('Mission3/Analyse_logs', 'velocity_norm.png')
    error_out = os.path.join('Mission3/Analyse_logs', 'error_norm.png')
    goal_lat = 48.6302766
    goal_lon = 7.7892683
    draw_vel = False
    plot_speed = True
    plot_error = True

    parsed = parse_log(input_path)
    save_csv_grouped(out_csv, parsed)
    print('Saved grouped CSV to', out_csv)
    plot_positions(parsed, draw_vel=draw_vel, goal_lat=goal_lat, goal_lon=goal_lon, plot_speed=plot_speed, speed_out=speed_out, plot_error=plot_error, error_out=error_out)


if __name__ == '__main__':
    main()
