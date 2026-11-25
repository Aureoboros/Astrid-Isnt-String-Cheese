Code snapshots extracted from the current Java sources. Each PNG identifies its
source file and line numbers; snippets omit surrounding code and preserve the shown logic.

- [Odometry and field coordinates](01_odometry_coordinates.png)
- [Limelight horizontal servo aiming](02_limelight_servo_aiming.png)
- [Gate motor jam recovery](03_gate_jam_recovery.png)

Pinpoint performs the two-pod/IMU integration in hardware; the first image shows
the field math, calibration and active Pedro pose update/read calls. The second
shows the camera's horizontal aiming loop, rather than distance calculations.
The third shows continuous gate velocity jam recovery used by the gate test,
TeleTourney, and live Red/Blue autos.

To refresh after source changes, install Pillow and Pygments in a Python environment
and run `python doc/code_snapshots/render_snapshots.py` from the project root.
The renderer uses the macOS Menlo font. PNGs are 2280 pixels wide with readable
syntax highlighting and source line numbers.
