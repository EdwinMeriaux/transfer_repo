"""Render an annotated top-down MP4 from a native Gazebo mission trajectory."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument('--run', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--seconds', type=float, default=28.)
parser.add_argument('--fps', type=int, default=30)
args = parser.parse_args()
rows = list(csv.DictReader((args.run / 'trajectory.csv').open()))
summary = json.loads((args.run / 'summary.json').read_text())
time_s = np.array([float(row['time_s']) for row in rows])
x = np.array([float(row['x']) for row in rows])
y = np.array([float(row['y']) for row in rows])
yaw = np.array([float(row['yaw']) for row in rows])
speed = np.array([float(row['actual_u']) for row in rows])
stages = [row['stage'] for row in rows]
frames = max(2, round(args.seconds * args.fps))
indices = np.linspace(0, len(rows)-1, frames).astype(int)

margin = 4.
xmin, xmax = float(x.min()-margin), float(x.max()+margin)
ymin, ymax = float(y.min()-margin), float(y.max()+margin)
span = max(xmax-xmin, ymax-ymin)
cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
xmin, xmax = cx-span/2, cx+span/2
ymin, ymax = cy-span/2, cy+span/2

fig, ax = plt.subplots(figsize=(10, 10), dpi=120)
fig.patch.set_facecolor('#071c26')
ax.set_facecolor('#0b5063')
ax.set_aspect('equal')
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.grid(color='white', alpha=.14, linewidth=.7)
ax.tick_params(colors='#d7eef4')
for spine in ax.spines.values():
    spine.set_color('#7fc9d8')
ax.set_xlabel('local east / x (m)', color='#d7eef4')
ax.set_ylabel('local north / y (m)', color='#d7eef4')
title = '20 m out, 180° turn, return' if summary['mission'] == 'out-and-back' else '25 m to center, 12 m-diameter circle'
ax.set_title('BlueBoat PPO — Gazebo Harmonic\n' + title, color='white', fontsize=16, pad=14)

ax.plot(x, y, color='#70d6ff', alpha=.24, linewidth=2)
trail, = ax.plot([], [], color='#ffd166', linewidth=3)
start = ax.scatter([x[0]], [y[0]], marker='*', s=180, color='#8aff80', edgecolors='black', zorder=6, label='start')
if summary['mission'] == 'out-and-back':
    heading = yaw[0]
    target = np.array([x[0]+20*math.cos(heading), y[0]+20*math.sin(heading)])
    ax.plot([x[0], target[0]], [y[0], target[1]], '--', color='white', alpha=.6, label='requested line')
    ax.scatter([target[0]], [target[1]], s=90, facecolors='none', edgecolors='white', linewidths=2)
else:
    center = np.asarray(summary['reached_center_m'])
    ax.scatter([center[0]], [center[1]], marker='+', s=180, color='white', linewidths=2, label='circle center')
    ax.add_patch(Circle(center, 6., fill=False, linestyle='--', linewidth=2, edgecolor='white', alpha=.65))
ax.legend(loc='upper left', facecolor='#082d38', labelcolor='white')

boat = Polygon(np.zeros((3, 2)), closed=True, facecolor='#ff595e', edgecolor='white', linewidth=1.5, zorder=8)
ax.add_patch(boat)
status = ax.text(.02, .02, '', transform=ax.transAxes, color='white', fontsize=12,
                 bbox=dict(facecolor='#062a34', edgecolor='#7fc9d8', alpha=.9, boxstyle='round,pad=.5'))


def update(frame):
    index = indices[frame]
    trail.set_data(x[:index+1], y[:index+1])
    length, width = 1.25, .75
    shape = np.array([[length, 0], [-.55*length, width], [-.55*length, -width]])
    c, s = math.cos(yaw[index]), math.sin(yaw[index])
    rotated = shape @ np.array([[c, s], [-s, c]]) + [x[index], y[index]]
    boat.set_xy(rotated)
    status.set_text(
        f"sim time: {time_s[index]:6.1f} s\n"
        f"stage: {stages[index]}\n"
        f"forward speed: {speed[index]:.2f} m/s")
    return trail, boat, status


writer = animation.FFMpegWriter(fps=args.fps, codec='libx264', bitrate=2400,
                                 extra_args=['-pix_fmt', 'yuv420p', '-movflags', '+faststart'])
animation.FuncAnimation(fig, update, frames=frames, interval=1000/args.fps, blit=True).save(args.output, writer=writer)
plt.close(fig)
print(args.output)
