# CHRPS Visual Servo

Visual servoing application for the Franka Research 3 (FR3) robot using ViSP and an Intel RealSense camera.  
This project extends ViSP’s `servoFrankaIBVS` example with configurable tag size, adjustable desired distance, and lost-target recovery behavior when the AprilTag is not visible.

## Lost-target recovery

When the AprilTag is lost, the controller uses the
**last known image drift** of the tag to **turn directly toward where it was heading**.

### Logic
1. **Backoff window (default 3.5 s)**  
   Gently move backward to widen FOV.
2. **Biased turn window (default 3.5 s)**  
   Apply an angular velocity bias from the last centroid motion in pixels/sec:  
   - Horizontal drift → yaw/roll bias  
   - Vertical drift → pitch bias  
   The bias decays over time if the tag is still not found.

If the tag reappears, the timer resets and normal IBVS resumes.

### Tuning knobs (in `servoFrankaIBVS_CHRPS.cpp`)
- `k_ang` (px/s → rad/s gain) and optional `bias_boost` multiply how hard we turn.
- `max_angular` caps angular speed.
- `bias_window_secs` controls how long we keep the pure biased turn before decaying.
- `scan_backoff_secs` controls the initial backoff duration.
- Optional: a small `v_c[2] = -0.01` m/s during biased turn can help re-acquire.

Example safe settings:
```cpp
double k_ang = vpMath::rad(0.12);
double bias_boost = 1.4;
double max_angular = vpMath::rad(30);
double bias_window_secs = 3.5;
double scan_backoff_secs = 3.5;


## 📦 Dependencies

- [libfranka](https://frankaemika.github.io/docs/installation_linux.html) (for FR3)
- [ViSP](https://visp.inria.fr) >= 3.6, built with RealSense and Franka support
- [Intel RealSense SDK 2.x](https://github.com/IntelRealSense/librealsense)
- CMake >= 3.10
- A C++17 compiler (GCC >= 9 recommended)

---

## 🔧 Building ViSP (once)

```bash
# Clone ViSP
git clone https://github.com/lagadic/visp.git
cd visp
mkdir build && cd build

# Configure + build + install
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/visp_install -DBUILD_EXAMPLES=OFF -DBUILD_DEMOS=OFF
make -j$(nproc)
make install
```

This will install ViSP into `~/visp_install`.

---

## 🚀 Building CHRPS Visual Servo

```bash
git clone https://github.com/ulubilgeulusoy/chrps_visual_servo.git
cd chrps_visual_servo
mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DViSP_DIR=~/visp_install/lib/cmake/visp
make -j$(nproc)
```

---

## ▶️ Running

Example run (FR3 connected at `172.16.0.2`):

```bash
./servoFrankaIBVS_CHRPS \
  --eMc config/eMc.yaml \
  --ip 172.16.0.2 \
  --tag-size 0.05 \
  --desired-factor 6 \
  --adaptive-gain \
  --plot
```

- `--tag-size` sets the physical AprilTag size in meters (default `0.05 m`)
- `--desired-factor` sets how many times the tag size is used to define desired Z distance (default `9`)
- `--eMc` provides the camera-to-end-effector calibration file
- `--adaptive-gain` improves convergence
- `--plot` shows feature and velocity curves

---

## Simple Build and Run (alternative)

./run_visual_servo_CHRPS.sh

## 📂 Project Structure

```
chrps_visual_servo/
│
├── src/
│   └── servoFrankaIBVS_CHRPS.cpp    # main application
├── config/
│   └── eMc.yaml                     # sample extrinsic calibration file
├── CMakeLists.txt
└── README.md
```

---

## 📝 Notes

- Default tag family is `36h11`.
- The robot must be in **velocity control mode** and connected before running.
- If no tag is detected, the robot moves back briefly to try to reacquire it.

---

## ⚖️ License

This project uses the GPLv2 license (from ViSP). See [LICENSE](LICENSE) for details.
