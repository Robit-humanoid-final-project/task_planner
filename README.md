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
colcon build --packages-select task_planner
source /opt/ros/jazzy/setup.bash
ros2 launch task_planner task_planner.launch.py
```

```
# gamecontroller
cd ~/colcon_ws
colcon build --packages-select gamecontroller
source /opt/ros/jazzy/setup.bash
ros2 launch gamecontroller gamecontroller.launch.py
```

## 로봇별 설정

다른 하나의 로봇에서 tp.yaml의 robot_id를 2로 변경

robot 1

```yaml
/**:
  ros__parameters:
    robot_id: 1 # 서로 다르게 설정
```

robot 2

```yaml
/**:
  ros__parameters:
    robot_id: 2 # 서로 다르게 설정
```
