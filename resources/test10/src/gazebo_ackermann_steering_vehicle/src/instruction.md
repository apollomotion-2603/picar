```bash
ros2 topic pub /my_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

---
Giờ có thể thay đổi tốc độ lúc runtime mà không cần rebuild:
# Set lúc launch
```
ros2 run my_bot ... --ros-args -p linear_speed:=1.5
```
# Hoặc thay đổi khi đang chạy
```
ros2 param set /gazebo linear_speed 2.0
```