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

```
navigation_pkg/
└── scripts/
    └── ftg_navigation.py   # Self-contained FTG node
run.py                      # BARN harness (launches Gazebo + FTG node)
Singularityfile.def         # Container definition
```

---

## Updates:
* 01/15/2025: Adding support for ROS2 in [The-Barn-Challenge-Ros2](https://github.com/Saadmaghani/The-Barn-Challenge-Ros2).
* 02/04/2024: Adding 60 [DynaBARN](https://github.com/aninair1905/DynaBARN) environments. DynaBARN environments can be accessed by world indexes from 300-359.

## Requirements
If you run it on a local machine without containers:
* ROS version at least Kinetic
* CMake version at least 3.0.2
* Python version at least 3.6
* Python packages: defusedxml, rospkg, netifaces, numpy

If you run it in Singularity containers:
* Go version at least 1.13
* Singularity version at least 3.6.3 and less than 4.02

The requirements above are just suggestions. If you run into any issue, please contact organizers for help (zfxu@utexas.edu).

## Installation
Follow the instructions below to run simulations on your local machines. (You can skip 1-6 if you only use Singularity container)

1. Create a virtual environment (we show examples with python venv, you can use conda instead)
```
apt -y update; apt-get -y install python3-venv
python3 -m venv /<YOUR_HOME_DIR>/nav_challenge
export PATH="/<YOUR_HOME_DIR>/nav_challenge/bin:$PATH"
```

2. Install Python dependencies
```
pip3 install defusedxml rospkg netifaces numpy
```

3. Create ROS workspace
```
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```

4. Clone this repo and required ros packages: (replace `<YOUR_ROS_VERSION>` with your own, e.g. melodic)
```
git clone https://github.com/Daffan/the-barn-challenge.git
git clone https://github.com/jackal/jackal.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_simulator.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_desktop.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/utexas-bwi/eband_local_planner.git
```

5. Install ROS package dependencies: (replace `<YOUR_ROS_VERSION>` with your own, e.g. melodic)
```
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash
rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=<YOUR_ROS_VERSION>
```

6. Build the workspace (if `catkin_make` fails, try changing `-std=c++11` to `-std=c++17` in `jackal_helper/CMakeLists.txt` line 3)
```
catkin_make
source devel/setup.bash
```

Follow the instruction below to run simulations in Singularity containers.

1. Follow this instruction to install Singularity: https://sylabs.io/guides/3.0/user-guide/installation.html. Singularity version >= 3.6.3 and <= 4.02 is required to successfully build the image!

2. Clone this repo
```
git clone https://github.com/Daffan/the-barn-challenge.git
cd the-barn-challenge
```

3. Build Singularity image (sudo access required)
```
sudo singularity build --notest nav_competition_image.sif Singularityfile.def
```

## Run Simulations
Navigate to the folder of this repo. Below is the example to run move_base with DWA as local planner.

If you run it on your local machines: (the example below runs [move_base](http://wiki.ros.org/move_base) with DWA local planner in world 0)
```
source ../../devel/setup.sh
python3 run.py --world_idx 0
```

If you run it in a Singularity container:
```
./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0
```

A successful run should print the episode status (collided/succeeded/timeout) and the time cost in second:
> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation collided with time 27.2930 (s)

> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
> Navigation succeeded with time 29.4610 (s)


> \>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<
>
>Navigation timeout with time 100.0000 (s)

## Test your own navigation stack
We currently don't provide a lot of instructions or a standard API for implementing the navigation stack, but we might add more in this section depending on people's feedback. If you are new to the ROS or mobile robot navigation, we suggest checking [move_base](http://wiki.ros.org/move_base) which provides basic interface to manipulate a robot.

The suggested work flow is to edit section 1 in `run.py` file (line 89-109) that initialize your own navigation stack. You should not edit other parts in this file. We provide a bash script `test.sh` to run your navigation stack on 50 uniformly sampled BARN worlds with 10 runs for each world. Once the tests finish, run `python report_test.py --out_path /path/to/out/file` to report the test. Below is an example of DWA:
```
python report_test.py --out_path res/dwa_out.txt
```
You should see the report as this:
>Avg Time: 33.4715, Avg Metric: 0.1693, Avg Success: 0.8800, Avg Collision: 0.0480, Avg Timeout: 0.0720

Except for `DWA`, we also provide three learning-based navigation stack as examples (see branch `LfH`, `applr` and `e2e`).

## Submission
Submit a link that downloads your customized repository to this [Google form](https://docs.google.com/forms/d/e/1FAIpQLSfZLMVluXE-HWnV9lNP00LuBi3e9HFOeLi30p9tsHUViWpqrA/viewform). Your navigation stack will be tested in the Singularity container on 50 hold-out BARN worlds sampled from the same distribution as the 300 BARN worlds. In the repository, make sure the `run.py` runs your navigation stack and `Singularityfile.def` installs all the dependencies of your repo. We suggest to actually build an image and test it with `./singularity_run.sh /path/to/image/file python3 run.py --world_idx 0`. You can also refer to branch `LfH`, `applr` and `e2e`, which are in the correct form for submissions.
