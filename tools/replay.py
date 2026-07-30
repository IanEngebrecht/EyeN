#!/usr/bin/env python3
"""
EyeAN log visualizer — replay from file or stream live from serial.

Usage:
    python replay.py <logfile.txt>                         # replay from file
    python replay.py <logfile.txt> --speed 2.0             # 2x playback
    python replay.py <logfile.txt> --speed 0               # step with spacebar
    python replay.py --live /dev/ttyUSB0                   # live from serial
    python replay.py --live /dev/ttyUSB0 --baud 115200     # custom baud

Reads $FRAME and $LOST lines emitted by EyeAN firmware.
In live mode, sliders let you tune LD2450 sensor parameters on the fly.
"""

import argparse
import collections
import re
import sys
import time

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Slider
import numpy as np

# ── Log parsing ──────────────────────────────────────────────────────────────

FRAME_RE = re.compile(r"\$FRAME\s+t=(\d+)\s+eye=(-?\d+),(-?\d+)(.*)")
LOST_RE = re.compile(r"\$LOST\s+t=(\d+)")
SENSOR_RE = re.compile(r"(\w+)=(\d+):([\S]+)")
TARGET_RE = re.compile(r"(-?\d+),(-?\d+),(\d+),(-?\d+)")

SENSOR_COLORS = ["#3498db", "#e74c3c", "#2ecc71", "#f39c12"]

ZONE_COLORS = {
    "upper":  "#e74c3c",
    "both":   "#f1c40f",
    "lower":  "#3498db",
}
ZONE_DIM = "#333333"


def parse_sensor_targets(blob):
    targets = []
    for part in blob.split("|"):
        m = TARGET_RE.match(part)
        if m:
            targets.append(
                dict(x=int(m[1]), y=int(m[2]), dist=int(m[3]), speed=int(m[4]))
            )
    return targets


def parse_line(line):
    m = LOST_RE.search(line)
    if m:
        return dict(t_ms=int(m[1]), eye_x=120, eye_y=120,
                    sensors={}, lost=True)
    m = FRAME_RE.search(line)
    if not m:
        return None
    t_ms = int(m[1])
    eye_x, eye_y = int(m[2]), int(m[3])
    rest = m[4]
    sensors = {}
    for sm in SENSOR_RE.finditer(rest):
        name, count = sm[1], int(sm[2])
        targets = parse_sensor_targets(sm[3])
        sensors[name] = dict(count=count, targets=targets)
    return dict(t_ms=t_ms, eye_x=eye_x, eye_y=eye_y,
                sensors=sensors, lost=False)


def parse_log(path):
    frames = []
    with open(path) as f:
        for line in f:
            frame = parse_line(line)
            if frame:
                frames.append(frame)
    return frames


# ── Zone logic ───────────────────────────────────────────────────────────────

def get_zone_counts(frame):
    """Derive per-zone target counts from sensor data.

    Returns dict with keys 'upper', 'both', 'lower'.
    Values are 0..3 representing detection intensity.
    Also returns the active band name (or None).
    """
    names = list(frame["sensors"].keys())
    if len(names) < 2:
        lo_n = frame["sensors"][names[0]]["count"] if names else 0
        return {"upper": 0, "both": 0, "lower": lo_n}, ("lower" if lo_n else None)

    lo_name, hi_name = names[0], names[1]
    lo_n = frame["sensors"][lo_name]["count"]
    hi_n = frame["sensors"][hi_name]["count"]

    zones = {"upper": 0, "both": 0, "lower": 0}
    active = None

    if lo_n > 0 and hi_n > 0:
        zones["both"] = min(lo_n, hi_n)
        zones["lower"] = lo_n
        zones["upper"] = hi_n
        active = "both"
    elif lo_n > 0:
        zones["lower"] = lo_n
        active = "lower"
    elif hi_n > 0:
        zones["upper"] = hi_n
        active = "upper"

    return zones, active


# ── Visualization ────────────────────────────────────────────────────────────

LCD_RES = 240
RADAR_MAX_MM = 6000
TRAIL_SECONDS = 30
LIVE_DEFAULT_RANGE = 2000


def setup_figure(radar_range_x=RADAR_MAX_MM, radar_range_y=RADAR_MAX_MM):
    fig = plt.figure(figsize=(16, 6.5))
    fig.patch.set_facecolor("#1e1e1e")

    gs = GridSpec(1, 3, figure=fig, width_ratios=[1.5, 0.18, 0.9],
                  wspace=0.25, left=0.05, right=0.96, bottom=0.1, top=0.92)
    ax_radar = fig.add_subplot(gs[0, 0])
    ax_zone = fig.add_subplot(gs[0, 1])
    ax_eye = fig.add_subplot(gs[0, 2])

    # ── Radar ──
    ax_radar.set_facecolor("#2d2d2d")
    ax_radar.set_xlim(-radar_range_x, radar_range_x)
    ax_radar.set_ylim(0, radar_range_y)
    ax_radar.set_xlabel("X  (mm)  ← left      right →", color="white", fontsize=9)
    ax_radar.set_ylabel("|Y|  (mm)  depth →", color="white", fontsize=9)
    ax_radar.set_title("Radar – top-down view", color="white", fontsize=11)
    ax_radar.tick_params(colors="gray", labelsize=8)
    for spine in ax_radar.spines.values():
        spine.set_color("gray")
    ax_radar.set_aspect("equal")
    ax_radar.grid(True, color="#444", linewidth=0.3)

    fov_half = np.radians(60)
    fov_r = max(radar_range_x, radar_range_y)
    for sign in [-1, 1]:
        fx = sign * fov_r * np.sin(fov_half)
        fy = fov_r * np.cos(fov_half)
        ax_radar.plot([0, fx], [0, fy], "--", color="#555", linewidth=0.8)
    ax_radar.plot(0, 0, "^", color="white", markersize=10, zorder=10)

    # ── Zone indicator (static frame) ──
    ax_zone.set_facecolor("#1e1e1e")
    ax_zone.set_xlim(0, 1)
    ax_zone.set_ylim(0, 1)
    ax_zone.set_xticks([])
    ax_zone.set_yticks([])
    for spine in ax_zone.spines.values():
        spine.set_visible(False)
    ax_zone.set_title("Bands", color="white", fontsize=10)

    # ── Eye ──
    ax_eye.set_facecolor("#2d2d2d")
    ax_eye.set_xlim(-10, LCD_RES + 10)
    ax_eye.set_ylim(LCD_RES + 10, -10)
    ax_eye.set_aspect("equal")
    ax_eye.set_title("Eye display", color="white", fontsize=11)
    ax_eye.tick_params(colors="gray", labelsize=8)
    for spine in ax_eye.spines.values():
        spine.set_color("gray")

    eye_circle = patches.Circle((LCD_RES / 2, LCD_RES / 2), LCD_RES / 2,
                                fill=True, facecolor="white",
                                edgecolor="#888", linewidth=1.5)
    ax_eye.add_patch(eye_circle)

    timestamp_txt = fig.text(0.5, 0.015, "", ha="center", color="gray", fontsize=9)

    return fig, ax_radar, ax_zone, ax_eye, timestamp_txt


def draw_zones(ax_zone, zones, active):
    """Draw three glowing zone bars with an arrow on the active band."""
    while ax_zone.patches:
        ax_zone.patches[-1].remove()
    while ax_zone.texts:
        ax_zone.texts[-1].remove()
    while ax_zone.lines:
        ax_zone.lines[-1].remove()
    while ax_zone.collections:
        ax_zone.collections[-1].remove()

    zone_order = ["upper", "both", "lower"]
    gap = 0.04
    h = (1.0 - gap * 4) / 3.0
    margin_x = 0.08
    bar_w = 1.0 - margin_x * 2

    for i, zname in enumerate(zone_order):
        y = 1.0 - gap * (i + 1) - h * (i + 1)
        count = zones.get(zname, 0)
        is_active = (zname == active)

        intensity = min(count / 3.0, 1.0)
        base_color = ZONE_COLORS[zname]

        # Background (dim)
        bg = patches.FancyBboxPatch(
            (margin_x, y), bar_w, h,
            boxstyle="round,pad=0.02",
            facecolor=ZONE_DIM, edgecolor="#555", linewidth=0.8)
        ax_zone.add_patch(bg)

        if intensity > 0:
            r, g, b = [int(base_color[i:i+2], 16) / 255 for i in (1, 3, 5)]
            fill_color = (r, g, b, 0.3 + 0.7 * intensity)
            fill = patches.FancyBboxPatch(
                (margin_x, y), bar_w * intensity, h,
                boxstyle="round,pad=0.02",
                facecolor=fill_color, edgecolor="none")
            ax_zone.add_patch(fill)

        if is_active:
            border = patches.FancyBboxPatch(
                (margin_x, y), bar_w, h,
                boxstyle="round,pad=0.02",
                facecolor="none", edgecolor=base_color, linewidth=2.5)
            ax_zone.add_patch(border)
            ax_zone.annotate("►", (margin_x - 0.02, y + h / 2),
                             fontsize=10, color=base_color, fontweight="bold",
                             ha="right", va="center")

        label = {"upper": "upper", "both": "both", "lower": "lower"}[zname]
        ax_zone.text(0.5, y + h / 2, label,
                     ha="center", va="center",
                     fontsize=8, color="white" if is_active else "#888",
                     fontweight="bold" if is_active else "normal")


def draw_frame(ax_radar, ax_zone, ax_eye, timestamp_txt, frame, idx, total, frames):
    while len(ax_radar.lines) > 3:
        ax_radar.lines[-1].remove()
    while ax_radar.collections:
        ax_radar.collections[-1].remove()
    while ax_radar.texts:
        ax_radar.texts[-1].remove()
    while ax_eye.lines:
        ax_eye.lines[-1].remove()
    while ax_eye.collections:
        ax_eye.collections[-1].remove()

    t_s = frame["t_ms"] / 1000.0
    status = "LOST" if frame["lost"] else "TRACKING"
    timestamp_txt.set_text(
        f"t = {t_s:.1f}s   frame {idx+1}/{total}   [{status}]"
    )

    # ── Zone indicator ──
    if frame["lost"]:
        draw_zones(ax_zone, {"upper": 0, "both": 0, "lower": 0}, None)
    else:
        zones, active = get_zone_counts(frame)
        draw_zones(ax_zone, zones, active)

    # ── Radar: sensor legend colors ──
    legend_entries = {}
    for si, name in enumerate(frame["sensors"]):
        legend_entries[name] = SENSOR_COLORS[si % len(SENSOR_COLORS)]

    # ── Trail ──
    now_ms = frame["t_ms"]
    trail_ms = TRAIL_SECONDS * 1000
    for ti in range(idx - 1, -1, -1):
        past = frames[ti]
        age_ms = now_ms - past["t_ms"]
        if age_ms > trail_ms:
            break
        frac = age_ms / trail_ms
        alpha = max(0.06, 0.4 * (1.0 - frac))
        size = max(10, 30 * (1.0 - frac * 0.7))
        for si, (name, sdata) in enumerate(past["sensors"].items()):
            color = SENSOR_COLORS[si % len(SENSOR_COLORS)]
            if name not in legend_entries:
                legend_entries[name] = color
            for tgt in sdata["targets"]:
                px, py = tgt["x"], abs(tgt["y"])
                ax_radar.scatter(px, py, c=color, s=size, alpha=alpha,
                                 edgecolors="none", zorder=3)

    # ── Current targets ──
    for si, (name, sdata) in enumerate(frame["sensors"].items()):
        color = SENSOR_COLORS[si % len(SENSOR_COLORS)]
        for tgt in sdata["targets"]:
            px, py = tgt["x"], abs(tgt["y"])
            ax_radar.scatter(px, py, c=color, s=90, alpha=0.95,
                             edgecolors="white", linewidths=0.8, zorder=5)
            label = f'{tgt["dist"]}mm'
            ax_radar.annotate(label, (px, py),
                              textcoords="offset points", xytext=(8, 8),
                              fontsize=7, color=color, alpha=0.95,
                              fontweight="bold")

    # ── Legend ──
    xlims = ax_radar.get_xlim()
    ylims = ax_radar.get_ylim()
    lx = xlims[0] + (xlims[1] - xlims[0]) * 0.05
    ly = ylims[1] * 0.95
    step_y = (ylims[1] - ylims[0]) * 0.06
    for i, (name, color) in enumerate(legend_entries.items()):
        ax_radar.scatter(lx, ly - i * step_y, c=color, s=50, zorder=6)
        ax_radar.annotate(name, (lx, ly - i * step_y),
                          textcoords="offset points", xytext=(12, -4),
                          fontsize=9, color=color, fontweight="bold")

    # ── Eye dot ──
    ex, ey = frame["eye_x"], frame["eye_y"]
    ax_eye.scatter(ex, ey, c="black", s=200, zorder=5)


# ── Auto-scale for live mode ─────────────────────────────────────────────────

def maybe_rescale(ax_radar, frames):
    xlims = list(ax_radar.get_xlim())
    ylims = list(ax_radar.get_ylim())
    changed = False
    for f in frames:
        for sdata in f["sensors"].values():
            for tgt in sdata["targets"]:
                ax_, ay = abs(tgt["x"]), abs(tgt["y"])
                if ax_ * 1.05 > xlims[1]:
                    xlims[0] = -ax_ * 1.2
                    xlims[1] = ax_ * 1.2
                    changed = True
                if ay * 1.05 > ylims[1]:
                    ylims[1] = ay * 1.2
                    changed = True
    if changed:
        ax_radar.set_xlim(xlims)
        ax_radar.set_ylim(ylims)
        fov_half = np.radians(60)
        fov_r = max(xlims[1], ylims[1])
        while len(ax_radar.lines) > 1:
            ax_radar.lines[0].remove()
        for sign in [-1, 1]:
            fx = sign * fov_r * np.sin(fov_half)
            fy = fov_r * np.cos(fov_half)
            ax_radar.plot([0, fx], [0, fy], "--", color="#555", linewidth=0.8)
        ax_radar.plot(0, 0, "^", color="white", markersize=10, zorder=10)


# ── Replay mode ──────────────────────────────────────────────────────────────

def compute_radar_range(frames):
    max_abs_x, max_abs_y = 500, 500
    for f in frames:
        for sdata in f["sensors"].values():
            for tgt in sdata["targets"]:
                max_abs_x = max(max_abs_x, abs(tgt["x"]))
                max_abs_y = max(max_abs_y, abs(tgt["y"]))
    pad = 1.2
    return int(max_abs_x * pad), int(max_abs_y * pad)


def run_replay(frames, speed):
    if not frames:
        print("No $FRAME or $LOST lines found in log file.", file=sys.stderr)
        sys.exit(1)

    rx, ry = compute_radar_range(frames)
    fig, ax_radar, ax_zone, ax_eye, ts_txt = setup_figure(
        radar_range_x=rx, radar_range_y=ry)

    paused = [False]
    step_forward = [False]

    def on_key(event):
        if event.key == " ":
            if speed == 0:
                step_forward[0] = True
            else:
                paused[0] = not paused[0]
        elif event.key == "right":
            step_forward[0] = True
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.ion()
    plt.show()

    t0_log = frames[0]["t_ms"]
    t0_wall = time.monotonic()

    i = 0
    while i < len(frames) and plt.fignum_exists(fig.number):
        if speed == 0:
            draw_frame(ax_radar, ax_zone, ax_eye, ts_txt,
                       frames[i], i, len(frames), frames)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            step_forward[0] = False
            while not step_forward[0] and plt.fignum_exists(fig.number):
                fig.canvas.flush_events()
                time.sleep(0.05)
            i += 1
            continue

        if paused[0] and not step_forward[0]:
            fig.canvas.flush_events()
            time.sleep(0.05)
            continue
        step_forward[0] = False

        target_wall = t0_wall + (frames[i]["t_ms"] - t0_log) / 1000.0 / speed
        now = time.monotonic()
        if now < target_wall:
            fig.canvas.flush_events()
            time.sleep(min(target_wall - now, 0.01))
            continue

        draw_frame(ax_radar, ax_zone, ax_eye, ts_txt,
                   frames[i], i, len(frames), frames)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        i += 1

    if plt.fignum_exists(fig.number):
        plt.ioff()
        plt.show()


# ── Live mode ────────────────────────────────────────────────────────────────

SLIDER_DEFS = [
    ("min_speed",   0,    50,    0, "%d",   "Min speed (cm/s)"),
    ("min_dist",    0,   500,  100, "%d",   "Min distance (mm)"),
    ("max_dist",  500,  6000, 4000, "%d",   "Max distance (mm)"),
    ("persist",     1,     3,    2, "%d",   "Persist (frames)"),
]


def send_cmd(ser, param, value):
    cmd = f"$SET {param} {int(value)}\n"
    try:
        ser.write(cmd.encode("utf-8"))
        ser.flush()
    except Exception as e:
        print(f"Send failed: {e}", file=sys.stderr)


def setup_live_figure():
    """Create figure with radar, zone, eye panels + bottom slider area."""
    n_sliders = len(SLIDER_DEFS)
    slider_height = 0.035
    slider_gap = 0.015
    slider_total = n_sliders * (slider_height + slider_gap) + 0.03
    plot_bottom = slider_total + 0.06

    fig = plt.figure(figsize=(16, 7.5))
    fig.patch.set_facecolor("#1e1e1e")

    gs = GridSpec(1, 3, figure=fig, width_ratios=[1.5, 0.18, 0.9],
                  wspace=0.25, left=0.05, right=0.96,
                  bottom=plot_bottom, top=0.92)
    ax_radar = fig.add_subplot(gs[0, 0])
    ax_zone = fig.add_subplot(gs[0, 1])
    ax_eye = fig.add_subplot(gs[0, 2])

    # ── Radar ──
    ax_radar.set_facecolor("#2d2d2d")
    ax_radar.set_xlim(-LIVE_DEFAULT_RANGE, LIVE_DEFAULT_RANGE)
    ax_radar.set_ylim(0, LIVE_DEFAULT_RANGE)
    ax_radar.set_xlabel("X  (mm)  ← left      right →", color="white", fontsize=9)
    ax_radar.set_ylabel("|Y|  (mm)  depth →", color="white", fontsize=9)
    ax_radar.set_title("Radar – top-down view", color="white", fontsize=11)
    ax_radar.tick_params(colors="gray", labelsize=8)
    for spine in ax_radar.spines.values():
        spine.set_color("gray")
    ax_radar.set_aspect("equal")
    ax_radar.grid(True, color="#444", linewidth=0.3)

    fov_half = np.radians(60)
    fov_r = LIVE_DEFAULT_RANGE
    for sign in [-1, 1]:
        fx = sign * fov_r * np.sin(fov_half)
        fy = fov_r * np.cos(fov_half)
        ax_radar.plot([0, fx], [0, fy], "--", color="#555", linewidth=0.8)
    ax_radar.plot(0, 0, "^", color="white", markersize=10, zorder=10)

    # ── Zone indicator ──
    ax_zone.set_facecolor("#1e1e1e")
    ax_zone.set_xlim(0, 1)
    ax_zone.set_ylim(0, 1)
    ax_zone.set_xticks([])
    ax_zone.set_yticks([])
    for spine in ax_zone.spines.values():
        spine.set_visible(False)
    ax_zone.set_title("Bands", color="white", fontsize=10)

    # ── Eye ──
    ax_eye.set_facecolor("#2d2d2d")
    ax_eye.set_xlim(-10, LCD_RES + 10)
    ax_eye.set_ylim(LCD_RES + 10, -10)
    ax_eye.set_aspect("equal")
    ax_eye.set_title("Eye display", color="white", fontsize=11)
    ax_eye.tick_params(colors="gray", labelsize=8)
    for spine in ax_eye.spines.values():
        spine.set_color("gray")

    eye_circle = patches.Circle((LCD_RES / 2, LCD_RES / 2), LCD_RES / 2,
                                fill=True, facecolor="white",
                                edgecolor="#888", linewidth=1.5)
    ax_eye.add_patch(eye_circle)

    timestamp_txt = fig.text(0.5, plot_bottom - 0.025, "", ha="center",
                             color="gray", fontsize=9)

    # ── Sliders ──
    sliders = {}
    for i, (param, vmin, vmax, vinit, fmt, label) in enumerate(SLIDER_DEFS):
        y = 0.03 + i * (slider_height + slider_gap)
        ax_s = fig.add_axes([0.15, y, 0.70, slider_height],
                            facecolor="#2d2d2d")
        sl = Slider(ax_s, label, vmin, vmax, valinit=vinit,
                    valstep=1, valfmt=fmt, color="#3498db",
                    initcolor="none")
        sl.label.set_color("white")
        sl.label.set_fontsize(8)
        sl.valtext.set_color("white")
        sl.valtext.set_fontsize(8)
        sliders[param] = sl

    return fig, ax_radar, ax_zone, ax_eye, timestamp_txt, sliders


def run_live(port, baud):
    try:
        import serial
    except ImportError:
        print("Live mode requires pyserial:  pip install pyserial", file=sys.stderr)
        sys.exit(1)

    print(f"Opening {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=0.05)
    print("Connected. Waiting for $FRAME lines...  (Q to quit)")
    print("Use sliders to tune sensor parameters (requires TX wired).")

    fig, ax_radar, ax_zone, ax_eye, ts_txt, sliders = setup_live_figure()

    frames = collections.deque(maxlen=500)
    frame_count = [0]
    pending_cmds = {}
    cmd_cooldown = {}
    CMD_DEBOUNCE_S = 0.8

    def make_slider_cb(param_name):
        def cb(val):
            pending_cmds[param_name] = int(val)
        return cb

    for param, sl in sliders.items():
        sl.on_changed(make_slider_cb(param))

    def on_key(event):
        if event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.ion()
    plt.show()

    buf = ""
    while plt.fignum_exists(fig.number):
        # Send pending slider commands (debounced: only send the latest value,
        # and wait CMD_DEBOUNCE_S between sends for each parameter)
        now = time.monotonic()
        sent = []
        for param, val in pending_cmds.items():
            last = cmd_cooldown.get(param, 0)
            if now - last >= CMD_DEBOUNCE_S:
                send_cmd(ser, param, val)
                cmd_cooldown[param] = now
                sent.append(param)
        for p in sent:
            del pending_cmds[p]

        try:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk.decode("utf-8", errors="replace")
        except Exception:
            pass

        new_frame = False
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            stripped = line.strip()
            if not stripped:
                continue
            frame = parse_line(stripped)
            if frame:
                frames.append(frame)
                frame_count[0] += 1
                new_frame = True
            else:
                print(f"  [{stripped}]")

        if new_frame and frames:
            frames_list = list(frames)
            idx = len(frames_list) - 1
            maybe_rescale(ax_radar, frames_list[-5:])
            draw_frame(ax_radar, ax_zone, ax_eye, ts_txt,
                       frames_list[idx], idx, frame_count[0], frames_list)
            fig.canvas.draw_idle()

        fig.canvas.flush_events()
        if not new_frame:
            time.sleep(0.02)

    ser.close()
    print("Closed.")


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="EyeAN log visualizer")
    parser.add_argument("logfile", nargs="?", help="Log file for replay mode")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Replay speed (0 = step mode, default 1.0)")
    parser.add_argument("--live", metavar="PORT",
                        help="Serial port for live mode (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Serial baud rate (default 115200)")
    parser.add_argument("--trail", type=int, default=30,
                        help="Trail duration in seconds (default 30)")
    args = parser.parse_args()

    global TRAIL_SECONDS
    TRAIL_SECONDS = args.trail

    if args.live:
        run_live(args.live, args.baud)
    elif args.logfile:
        print(f"Parsing {args.logfile}...")
        frames = parse_log(args.logfile)
        print(f"Found {len(frames)} frames.")
        if args.speed == 0:
            print("Step mode: SPACE or → to advance, Q to quit.")
        else:
            print(f"Playing at {args.speed}× speed. SPACE to pause, → to step, Q to quit.")
        run_replay(frames, args.speed)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
