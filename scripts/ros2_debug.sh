# 位置
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"
# 关节阻抗
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"
# 空闲
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 0}"