"""
visualise_imu.py — Real-time 3-D orientation display for the MRV robot.

Reads the LSM6DSV16X SFLP quaternion from the Teensy serial output and
displays a live coordinate frame (RGB = XYZ axes) in a matplotlib 3-D plot.

Usage:
    python tools/visualise_imu.py
    python tools/visualise_imu.py /dev/cu.usbmodem187379501

Serial format expected (produced by SerialReporter):
    T:<us> | Q:<x>,<y>,<z>,<w> | RPY:<r>,<p>,<y> | J:...
"""

import re
import sys
import threading

import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3-D projection

# ── Configuration ─────────────────────────────────────────────────────────────
PORT    = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem187379501"
BAUD    = 115200
PLOT_HZ = 20   # matplotlib refresh rate

# ── Shared state (written by serial thread, read by plot) ─────────────────────
_lock      = threading.Lock()
_quat      = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w — identity
_rpy       = np.array([0.0, 0.0, 0.0])        # degrees
_connected = False

# ── Serial parsing ────────────────────────────────────────────────────────────
_QPAT   = re.compile(r'Q:([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)')
_RPYPAT = re.compile(r'RPY:([-\d.]+),([-\d.]+),([-\d.]+)')


def _serial_thread():
    global _connected
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            with _lock:
                _connected = True
            print(f"Connected to {PORT}")
            while True:
                raw = ser.readline()
                line = raw.decode('ascii', errors='ignore').strip()
                qm = _QPAT.search(line)
                rm = _RPYPAT.search(line)
                if qm and rm:
                    with _lock:
                        _quat[:] = (float(qm.group(1)), float(qm.group(2)),
                                    float(qm.group(3)), float(qm.group(4)))
                        _rpy[:]  = (float(rm.group(1)), float(rm.group(2)),
                                    float(rm.group(3)))
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)


# ── Quaternion → rotation matrix ──────────────────────────────────────────────
def _quat_to_rot(x, y, z, w):
    """Returns 3×3 rotation matrix from unit quaternion (Hamilton convention)."""
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


# ── Plot setup ────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(7, 7))
ax  = fig.add_subplot(111, projection='3d')
ax.set_title("MRV — IMU orientation  (LSM6DSV16X SFLP)", fontsize=11)

COLOURS = ['#e74c3c', '#2ecc71', '#3498db']  # X=red  Y=green  Z=blue
LABELS  = ['X', 'Y', 'Z']
ORIGIN  = np.zeros(3)

# Initial arrows along the identity axes; rebuilt each frame.
quivers = [
    ax.quiver(*ORIGIN, *np.eye(3)[i], color=COLOURS[i],
              linewidth=3, arrow_length_ratio=0.15, label=LABELS[i])
    for i in range(3)
]

status_txt = ax.text2D(0.02, 0.97, "", transform=ax.transAxes,
                        fontsize=9, va='top', family='monospace')
rpy_txt    = ax.text2D(0.02, 0.89, "", transform=ax.transAxes,
                        fontsize=9, va='top', family='monospace', color='#444444')

ax.set_xlim(-1.2, 1.2);  ax.set_xlabel('X')
ax.set_ylim(-1.2, 1.2);  ax.set_ylabel('Y')
ax.set_zlim(-1.2, 1.2);  ax.set_zlabel('Z')
ax.legend(loc='upper right', fontsize=9)
ax.set_box_aspect([1, 1, 1])


def _update(_frame):
    global quivers
    with _lock:
        q   = _quat.copy()
        rpy = _rpy.copy()
        ok  = _connected

    R = _quat_to_rot(*q)

    # matplotlib 3-D quivers have no in-place direction setter — remove & redraw.
    for qv in quivers:
        qv.remove()
    quivers = [
        ax.quiver(*ORIGIN, *R[:, i], color=COLOURS[i],
                  linewidth=3, arrow_length_ratio=0.15)
        for i in range(3)
    ]

    status_txt.set_text(
        f"{'● connected' if ok else '○ waiting…'}   "
        f"q = [{q[0]:+.4f}  {q[1]:+.4f}  {q[2]:+.4f}  {q[3]:+.4f}]"
    )
    rpy_txt.set_text(
        f"Roll {rpy[0]:+7.2f}°    Pitch {rpy[1]:+7.2f}°    Yaw {rpy[2]:+7.2f}°"
    )

    return quivers + [status_txt, rpy_txt]


ani = animation.FuncAnimation(
    fig, _update,
    interval=1000 // PLOT_HZ,
    blit=False,           # blit=True breaks quiver removal on 3-D axes
    cache_frame_data=False,
)

# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
    t = threading.Thread(target=_serial_thread, daemon=True)
    t.start()
    print(f"Connecting to {PORT} at {BAUD} baud…  Close the window to exit.")
    plt.tight_layout()
    plt.show()
