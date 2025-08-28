# CHRPS Visual Servo

Visual servoing application for the Franka Research 3 (FR3) robot using ViSP and an Intel RealSense camera.  
This project extends ViSP’s `servoFrankaIBVS` example with configurable tag size, adjustable desired distance, and basic recovery behavior when the AprilTag is not visible.

---

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
  --desired-factor 9 \
  --adaptive-gain \
  --plot
```

- `--tag-size` sets the physical AprilTag size in meters (default `0.05 m`)
- `--desired-factor` sets how many times the tag size is used to define desired Z distance (default `9`)
- `--eMc` provides the camera-to-end-effector calibration file
- `--adaptive-gain` improves convergence
- `--plot` shows feature and velocity curves

---

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
