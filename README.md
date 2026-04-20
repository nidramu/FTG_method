<p align="center">
  <img width = "100%" src='res/BARN_Challenge.png' />
</p>

--------------------------------------------------------------------------------

# ICRA BARN Navigation Challenge — Follow the Gap Submission

This repository contains a custom navigation stack for the ICRA BARN Challenge based on the **Follow the Gap (FTG)** algorithm. The implementation is purely reactive: it uses only 2D LiDAR data and odometry — no map, no global planner.

## Navigation Stack: Follow the Gap

### How it works

The algorithm runs at 10 Hz and follows these steps on each scan:

1. **LiDAR preprocessing** — raw ranges are clipped to `[min_safe_dist, max_dist]`, NaN/Inf values are replaced, and a 5-point moving average smooths the signal.
2. **Direct goal check** — if the direction toward the goal is clear and no obstacle is close, the robot drives straight toward the goal (no gap selection needed).
3. **Safety bubble** — a zero-range bubble is placed around the closest obstacle. Bubble radius scales inversely with obstacle distance, making it tighter for faraway obstacles.
4. **Gap finding** — contiguous segments of non-zero ranges above `min_gap_threshold` are extracted. Gaps narrower than the robot's physical width (`robot_width + safety_margin`) are discarded; if none pass, the widest available gap is used as a fallback.
5. **Gap scoring** — each valid gap is scored by a weighted combination of angular distance to the goal direction (weight 0.85) and gap tightness (weight 0.15). Shallow gaps get a penalty.
6. **Gap hysteresis** — to avoid oscillation, the previously selected gap is preferred when its score is within `gap_switch_score_margin` of the best new gap.
7. **Heading offset** — the target angle is nudged away from the closer wall of the chosen gap by a footprint offset (`atan2(robot_half_width, gap_depth)`).
8. **Smoothing** — the heading is exponentially smoothed (`alpha=0.5`), with a hard reset on direction reversals > 90°.
9. **Velocity control** — linear speed scales with proximity to the nearest obstacle (`[0.35, 1.8] m/s`) and is further reduced for tight gaps and large heading corrections.

### Key parameters

| Parameter | Value | Description |
|---|---|---|
| `max_dist` | 4.5 m | LiDAR range cap |
| `min_safe_dist` | 0.3 m | Ranges below this are zeroed |
| `robot_width` | 0.43 m | Jackal physical width |
| `safety_margin` | 0.25 m | Extra clearance added to robot width |
| `max_v` | 1.8 m/s | Maximum linear speed |
| `max_w` | 1.5 rad/s | Maximum angular speed |
| `min_gap_threshold` | 0.8 m | Minimum range for a cell to count as open |
| `smooth_alpha` | 0.5 | Heading exponential smoothing factor |

### File structure

```text
navigation_pkg/
└── scripts/
    └── ftg_navigation.py   # Self-contained FTG node
run.py                      # BARN harness (launches Gazebo + FTG node)
Dockerfile                  # Docker image definition
docker_run.sh               # Docker execution wrapper
Singularityfile.def         # Singularity definition
```

### Latest benchmark results (this project)

The latest evaluated variant is **FAZ 4 v5 (creep & spin dead-lock fix)**.

| Run | Worlds | Success | Collision | Timeout | Metric |
|---|---:|---:|---:|---:|---:|
| Smoke | 34 | 31/34 (**91.2%**) | 3 | 0 | **0.4280** |
| Full | 300 | 236/300 (**78.7%**) | 56 | 8 | **0.3128** |

Additional full-run signal:
- `ESC triggered` count: **39**

Result files:
- `faz4v5_smoke_20260420_065715.txt`
- `faz4v5_smoke_20260420_065715.log`
- `faz4v5_full300_20260420_072004.txt`
- `faz4v5_full300_20260420_072004.log`

Full-300 averages:
- **Avg Time: 17.2723**
- **Avg Metric: 0.3128**
- **Avg Success: 0.7867**
- **Avg Collision: 0.1867**
- **Avg Timeout: 0.0267**

---

## Docker-first setup (recommended)

### Requirements
- Docker Engine (Linux)
- VS Code (recommended)
- Optional VS Code extension: **Docker**

### Build image

```bash
docker build -t barn-ftg:latest .
```

### Run one world

```bash
./docker_run.sh barn-ftg:latest python3 run.py --world_idx 0
```

### Run one world and write output

```bash
./docker_run.sh barn-ftg:latest python3 run.py --world_idx 0 --out out.txt
```

Output row format:

```text
world_idx success collided timeout time nav_metric
```

### Run all 300 worlds once

```bash
./docker_run.sh barn-ftg:latest bash test_300.sh out_300.txt
```

### Run all 300 worlds with repeats

```bash
REPEATS=10 ./docker_run.sh barn-ftg:latest bash test_300.sh out_300x10.txt
```

---

## Optional: local (non-container) installation

If you run on a local machine without containers:
- ROS version at least Kinetic
- CMake version at least 3.0.2
- Python version at least 3.6
- Python packages: `defusedxml`, `rospkg`, `netifaces`, `numpy`

1. Create a virtual environment:
```bash
apt -y update; apt-get -y install python3-venv
python3 -m venv /<YOUR_HOME_DIR>/nav_challenge
export PATH="/<YOUR_HOME_DIR>/nav_challenge/bin:$PATH"
```

2. Install Python dependencies:
```bash
pip3 install defusedxml rospkg netifaces numpy
```

3. Create ROS workspace:
```bash
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```

4. Clone required repositories (replace `<YOUR_ROS_VERSION>` with your version, e.g. melodic):
```bash
git clone https://github.com/Daffan/the-barn-challenge.git
git clone https://github.com/jackal/jackal.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_simulator.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_desktop.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/utexas-bwi/eband_local_planner.git
```

5. Install ROS package dependencies:
```bash
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=<YOUR_ROS_VERSION>
```

6. Build workspace:
```bash
catkin_make
source devel/setup.bash
```

---

## Optional: Singularity

1. Install Singularity (>= 3.6.3 and <= 4.02).
2. Build image:
```bash
sudo singularity build --notest nav_competition_image.sif Singularityfile.def
```
3. Run:
```bash
./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0
```

---

## Run simulations

### Local machine
```bash
source ../../devel/setup.sh
python3 run.py --world_idx 0
```

### Docker
```bash
./docker_run.sh barn-ftg:latest python3 run.py --world_idx 0
```

### Singularity
```bash
./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0
```

---

## Test your own navigation stack

To plug in your own stack, edit the launch block in `run.py` (Section 1 around the nav process start) and keep the rest of `run.py` unchanged.

Example report command:
```bash
python report_test.py --out_path /path/to/out/file
```

---

## Troubleshooting

### `docker: permission denied`
```bash
sudo usermod -aG docker $USER
newgrp docker
```

### ROS/Gazebo communication issues
```bash
./docker_run.sh barn-ftg:latest env | grep -E 'ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME'
```

### Script permission errors
```bash
chmod +x docker_run.sh entrypoint.sh test_300.sh singularity_run.sh
```
