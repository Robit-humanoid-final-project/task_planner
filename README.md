# 빌드 & 실행

## 빌드

```
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

```
# task_planner
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
ros2 launch task_planner task_planner.launch.py
```
