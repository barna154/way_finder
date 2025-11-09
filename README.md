# Robot irányítás lidar segítségével iskolai projekhez
ROS 2 C++ package.  ![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)[![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/humble/)
## Packages and build
## Packages and Build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the Packages

```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/robotverseny/megoldas_sim24
```

### Rviz 2D Overlay

```bash
sudo apt install ros-humble-rviz-2d-overlay*
```

### Build ROS 2 Packages

```bash
cd ~/ros2_ws
```
```bash
colcon build --packages-select megoldas_sim24 --symlink-install
```

### Run the ROS 2 Packages

<details>
<summary>Don't forget to source before ROS commands.</summary>

```bash
source ~/ros2_ws/install/setup.bash
```
</details>

```bash
ros2 launch megoldas_sim24 megoldas1.launch.py # start simple_pursuit
```
```bash
ros2 run megoldas_sim24 simple_pursuit.py
```
```bash
ros2 launch megoldas_sim24 megoldas2.launch.py # start follow_the_gap
```
```bash
ros2 run megoldas_sim24 follow_the_gap.py
```

