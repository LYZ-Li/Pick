# Pick
```
colcon build --symlink-install --packages-select ur5e_pick_place_bringup
source install/setup.bash
```
```
ros2 launch ur5e_pick_place_bringup phase_b_calibration.launch.py launch_gripper:=false
```