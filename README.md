<div align="center">
  <h1 align="center">Dex Retargeting (Isaac Sim Version)</h1>
  <h3 align="center">
    Various retargeting optimizers to translate human hand motion to robot hand motion – adapted to NVIDIA Isaac Sim + ROS 2 + Vive/Leap teleop.
  </h3>
</div>

<p align="center">
  <!-- upstream badges (credit only) -->
  <a href="https://github.com/dexsuite/dex-retargeting/actions/workflows/test.yml">
    <img src="https://github.com/dexsuite/dex-retargeting/actions/workflows/test.yml/badge.svg" alt="Upstream Test Status" />
  </a>
  <a href="https://github.com/dexsuite/dex-retargeting/issues">
    <img src="https://img.shields.io/github/issues-closed/dexsuite/dex-retargeting.svg" alt="Issues Closed">
  </a>
  <a href="https://github.com/dexsuite/dex-retargeting/tags">
    <img src="https://img.shields.io/github/v/release/dexsuite/dex-retargeting.svg?include_prereleases&sort=semver" alt="Releases">
  </a>
  <a href="https://github.com/dexsuite/dex-retargeting/blob/main/LICENSE">
    <img alt="License" src="https://img.shields.io/badge/license-MIT-blue">
  </a>
</p>

<div align="center">
  <h4>This repository reuses and adapts code from the <a href="https://github.com/dexsuite/dex-retargeting">dexsuite/dex-retargeting</a> repo and the <a href="https://yzqin.github.io/anyteleop/">AnyTeleop Project</a>.</h4>
  <img src="example/vector_retargeting/teaser.webp" alt="Retargeting with different hands.">
</div>

---

## 0. About This Repo

- **Not a fork**: This repo was **reorganized and modified** for Isaac Sim workflows rather than forked directly.
- **Environment difference**: Everything is executed through Isaac Sim’s `python.sh`, not system Python or a normal venv.
- **Teleop stack**: Uses Vive Tracker (UDP publisher) + Leap Hand + ROS 2 Humble.
- **Original credit & license** are preserved below.

---

## 1. Installation (Isaac Sim Workflow)

### Requirements

- **Isaac Sim 4.5** (assumed at `~/isaacsim`)
- **ROS 2 Humble** already installed and sourced

#### 0) ROS 2 (one time)
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### 1) Isaac Sim dependencies
```bash
~/isaacsim/python.sh -m pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com
```
#### 2) Install THIS repo (from repo root)
```bash
cd ~/isaacsim/dex-retargeting-isaac
~/isaacsim/python.sh -m pip install -e ".[example]"
```
#### 3) (Compatibility fix) lock NumPy / SciPy
```bash
~/isaacsim/python.sh -m pip install "numpy==1.24.4" "scipy==1.10.1"
```
#### 4) motion generation extension
```bash
~/isaacsim/python.sh -m pip install isaacsim-robot-motion==4.5.0.0 --extra-index-url https://pypi.nvidia.com
```

---

## 2.1 Quick Test

Before jumping into the full teleoperation setup, we provide simple methods to verify that the system runs correctly without requiring a Vive Tracker.

### 1. Image-Based Teleoperation (Webcam, RealSense, etc.)

You can control the Leap Hand using visual input from a webcam, Intel RealSense, or other camera devices. This mode runs a hand pose estimation pipeline (e.g., via MediaPipe or a custom model) that maps detected hand landmarks to joint angles in simulation.

This method enables a more intuitive and natural control experience, ideal for live demos or testing human-robot hand imitation.

### 2. Move the Robot without Vive Tracker

We show how to move the entire robot (e.g., Franka FR3) using a simple ROS 2 publisher.

#### Trajectory: Sine Wave

We publish a sine wave trajectory to all 7 joints to simulate periodic arm movement. This helps verify:

- ROS 2 communication
- Joint control via `/fr3/joint_command` topic
- Isaac Sim's articulation response

#### Example Code: `fr3_publisher.py`



### How to Run

You will need **three terminals** (e.g., using Terminator split layout) to run the full teleoperation setup.

#### Step-by-Step

**Source ROS 2 environment in all terminals:**

```bash
source /opt/ros/humble/setup.bash
```

**1. Launch Isaac Sim + Retargeting Script (Terminal A)**

```bash
~/isaacsim/python.sh ~/isaacsim/dex-retargeting-isaac/example/vector_retargeting/denso_control_sim.py \
  --robot-name leap \
  --retargeting-type position \
  --hand-type right \
  --sim \
  --gui
```

**2. Run the Robot Publisher (Terminal B)**

```bash
python3 fr3_publisher.py
```

**3. Visualize with PlotJuggler (Terminal C)**

```bash
plotjuggler
```

&#x20;

📹 **Video Demo:** [Watch here](https://youtu.be/daJ74gNN8xI)

📌 **Tip:** Use [Terminator](https://gnometerminator.blogspot.com/p/introduction.html) to arrange all terminals in one screen for easier debugging and monitoring.

📌 **Note:** Some jitter in the robot's motion may be observed. This is a known issue under investigation.

---


## 2.2 Quick Start / Launch

This setup needs **three terminals**.

### Terminal A — Launch Isaac Sim GUI + Robot

```bash
~/isaacsim/python.sh ~/isaacsim/dex-retargeting-isaac/example/vector_retargeting/denso_control_sim.py   --robot-name leap   --retargeting-type position   --hand-type right   --sim   --gui
```

### Terminal B — Retargeting ROS 2 Node

```bash
~/isaacsim/python.sh ~/isaacsim/dex-retargeting-isaac/example/vector_retargeting/teleop_vive_leap_ros2.py
```

### Terminal C — Initial origin

```bash
ros2 service call /reset_tracking_origin std_srvs/srv/Trigger "{}"
```

### Terminal D — Vive Tracker UDP Publisher (on Windows env)

> ⚠️ The script `vive_publisher_udp.py` is **not provided in this repo** (it runs on Windows or Linux to broadcast tracker poses via UDP).

```bash
~/isaacsim/python.sh vive_publisher_udp.py
```

---

## 3. Differences from Upstream `dexsuite/dex-retargeting`

| Feature / Aspect          | Upstream (`dexsuite`)            | This Repo (`-isaac`)                           |
|---------------------------|----------------------------------|------------------------------------------------|
| Simulator / Renderer      | SAPIEN                           | Isaac Sim                                      |
| Python Environment        | Global pip / conda               | `~/isaacsim/python.sh`                         |
| Input Modality            | Video + MediaPipe / datasets     | Vive Tracker via UDP + Leap Hand               |
| ROS 2                     | Optional                         | Required (Humble)                              |
| GUI                       | Minimal / custom                 | Full Isaac GUI (`--gui`)                       |
| Numpy Version             | ≥2.0.0 (v0.5.0+)                  | Sticking to 1.24.4 for C++ ext compatibility   |
| Deployment                | `pip install dex_retargeting`    | `python.sh -m pip install -e .`                |

---

## 4. (Optional) Vanilla Installation (Upstream Style)

If you **do not** use Isaac Sim and just want the original package:

```bash
pip install dex_retargeting
# or dev mode
git clone https://github.com/dexsuite/dex-retargeting
cd dex-retargeting
pip install -e ".[example]"
```

See upstream README for MediaPipe / rendering extras.

---

## 5. FAQ & Troubleshooting

### 5.1 ModuleNotFoundError: `dex_retargeting`

- Ensure the package is installed **in Isaac Sim’s Python**:
  ```bash
  ~/isaacsim/python.sh -m pip install -e .
  ```
- Or prepend `PYTHONPATH=src` when running, but install is recommended.

### 5.2 NumPy / SciPy ABI Errors

```
A module compiled with NumPy 1.x cannot run on NumPy 2.x...
```

- Downgrade inside Isaac env:
  ```bash
  ~/isaacsim/python.sh -m pip install "numpy==1.24.4" "scipy==1.10.1"
  ```

### 5.3 Joint Order Mismatch

URDF parsers (ROS, simulators, drivers) may use different joint orders. Always map by **joint name**:

```python
from dex_retargeting.seq_retarget import SeqRetargeting
import numpy as np

retargeting: SeqRetargeting
sapien_joint_names = [joint.get_name() for joint in robot.get_active_joints()]
retargeting_joint_names = retargeting.joint_names
idx_map = np.array([retargeting_joint_names.index(n) for n in sapien_joint_names], dtype=int)

robot.set_qpos(retarget_qpos[idx_map])
```

> Replace `sapien_joint_names` / `robot.set_qpos` with Isaac Sim equivalents.

---

## 6. Repo Structure (simplified)

```
~/isaacsim/
├── apps/
├── ...
├── dex-retargeting-isaac/ ← This repository
│ ├── example/
│ ├── src/
│ └── ...
├── python.sh # Used for all Python execution
├── isaac-sim.sh
└── ...
```

---

## 7. Changelog (Local)

- **2025-07**: Initial Isaac Sim adaptation; switched to `python.sh` workflow, added Vive UDP pipeline.
- See upstream repo for full feature changelog: https://github.com/dexsuite/dex-retargeting

---

## 8. Citation

If you use this work, please cite the original AnyTeleop paper:

```bibtex
@inproceedings{qin2023anyteleop,
  title     = {AnyTeleop: A General Vision-Based Dexterous Robot Arm-Hand Teleoperation System},
  author    = {Qin, Yuzhe and Yang, Wei and Huang, Binghao and Van Wyk, Karl and Su, Hao and Wang, Xiaolong and Chao, Yu-Wei and Fox, Dieter},
  booktitle = {Robotics: Science and Systems},
  year      = {2023}
}
```

---

## 9. License

MIT License (inherited from upstream).  
Check `LICENSE` for details.

---

## 10. Acknowledgments

- Original repository: <a href="https://github.com/dexsuite/dex-retargeting">dexsuite/dex-retargeting</a><br>
- Hand URDFs: <a href="https://github.com/dexsuite/dex-urdf">dex-urdf</a><br>
- Kinematics: <a href="https://github.com/stack-of-tasks/pinocchio">pinocchio</a><br>
- Upstream examples used <a href="https://github.com/haosulab/SAPIEN">SAPIEN</a> for visualization.<br>
- This repo uses <b>NVIDIA Isaac Sim</b> instead.

