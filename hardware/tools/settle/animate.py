#!/usr/bin/env python3
"""settle: mp4 of a run from frames.json (settle.py --frames N).

    python3 animate.py CONFIG.json [--fps 12] [--zoom REF W] [--hold 12] [--stride K]

Writes review/img/settle_<name>.mp4 (whole region) and, with --zoom, the
window of W mm around ghost/part REF as settle_<name>_zoom_<REF>.mp4.
Every frame shows the copper, the parts, the ghosts at their current scale,
the pressure deposited that cycle (arrows on parts, dots on nodes), and the
clamps that stopped a unit (x). The last frame is held for --hold frames.
"""
import os, sys, math, json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from common import HW, load_config, load_json, data_path
from viz import state_from_frame, draw_state, legend_handles, fix_chains0

cfg = load_config()
M = load_json(cfg, 'model.json')
FR = fix_chains0(load_json(cfg, 'frames.json'), load_json(cfg, 'traj.json'))
name = cfg['name']
IMG = os.path.join(os.path.dirname(HW), 'review', 'img')
os.makedirs(IMG, exist_ok=True)


def arg(flag, default, cast=float):
    return cast(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default

FPS = arg('--fps', 12, int)
HOLD = arg('--hold', 12, int)
STRIDE = arg('--stride', 1, int)
zoom = None
if '--zoom' in sys.argv:
    i = sys.argv.index('--zoom')
    zoom = (sys.argv[i + 1], float(sys.argv[i + 2]))

frames = FR['frames'][::STRIDE]
if frames[-1] is not FR['frames'][-1]:
    frames.append(FR['frames'][-1])


def window_for(ref, w):
    for gh in M['ghosts']:
        if gh['ref'] == ref:
            cx, cy = gh['at']
            break
    else:
        p = M['parts'][ref]
        cx, cy = p['cx'], p['cy']
    return [cx - w / 2, cy - w / 2, cx + w / 2, cy + w / 2]


def render(out, window, figsize, title_prefix):
    fig, ax = plt.subplots(figsize=figsize)
    writer = FFMpegWriter(fps=FPS, codec='libx264', bitrate=-1,
                          extra_args=['-pix_fmt', 'yuv420p', '-crf', '20', '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2'])
    with writer.saving(fig, out, dpi=110):
        for k, fr in enumerate(frames):
            ax.cla()
            st = state_from_frame(M, FR, fr)
            born = sum(1 for v in st['G'] if v >= 1.0)
            ttl = (f"{title_prefix} cycle {st['cycle']}  progress {st['s']:.2f}  " +
                   (f"ghosts born {born}/{len(M['ghosts'])}  " if M['ghosts'] else '') +
                   f"parts moved {sum(1 for d in st['D'].values() if math.hypot(*d) > 1e-3)}")
            draw_state(ax, M, st, window=window, title=ttl, label_ghosts=True)
            ax.set_xlabel('mm', fontsize=8)
            writer.grab_frame()
            if k == len(frames) - 1:
                for _ in range(HOLD):
                    writer.grab_frame()
    plt.close(fig)
    print(f'wrote {out} ({len(frames)} frames @ {FPS} fps)')

reg = M['region']
aspect = (reg[3] - reg[1]) / (reg[2] - reg[0])
if zoom is None:
    w = 7.0 if aspect > 1 else 12.0
    render(os.path.join(IMG, f'settle_{name}.mp4'), None, (w, w * aspect + 0.6), name)
else:
    ref, wmm = zoom
    render(os.path.join(IMG, f'settle_{name}_zoom_{ref}.mp4'), window_for(ref, wmm), (7.5, 7.9), f'{name} · {ref}')
