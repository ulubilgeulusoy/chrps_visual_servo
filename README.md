# CHRPS Visual Servo

This repository contains a customized **ViSP-based visual servoing** example for the Franka Research 3 (Panda) robot.  
It extends the standard `servoFrankaIBVS` demo with additional features such as:

- **Configurable AprilTag size** (`--tag-size`)
- **Custom desired distance factor** (`--desired-factor`)
- **Fallback behavior**: if no tag is detected, the robot automatically steps back to try and find the tag again
- Clean, standalone CMake build (does not require building all of ViSP examples)

---

## Build Instructions

First make sure you have ViSP installed locally (with CMake config available):

```bash
cd ~/franka_ws/src/visp/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/visp_install -DBUILD_EXAMPLES=ON -DBUILD_DEMOS=ON
make -j$(nproc)
make install
```

Then build this project:

```bash
cd ~/chrps_visual_servo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=~/visp_install/lib/cmake/visp
make -j$(nproc)
```

---

## Running

Example usage:

```bash
cd ~/chrps_visual_servo/build
./servoFrankaIBVS_CHRPS \
  --eMc ../config/eMc.yaml \
  --ip 172.16.0.2 \
  --no-convergence-threshold \
  --adaptive-gain \
  --plot \
  --tag-size 0.05 \
  --desired-factor 9
```

You can also use the helper script:

```bash
./run_visual_servo_CHRPS.sh
```

---

## Arguments

- `--tag-size <m>` : AprilTag size in meters (default: `0.05`)
- `--desired-factor <n>` : desired distance as multiple of tag size (default: `9`)
- `--ip <addr>` : Franka controller IP (default: `192.168.1.1`)
- `--eMc <file>` : YAML file with camera extrinsic calibration
- `--adaptive-gain` : Enable adaptive control gain
- `--plot` : Show live plots of errors and velocities
