**<running guide for 2oth>**


**dynamixel_rdk** 
1. ros2 launch dynamixel_rdk_ros dynamixel_rdk.launch.py
2. ros2 lifecycle set dynamixel_rdk_node configure
3. 성공시 ros2 lifecycle set dynamixel_rdk_node activate


**ik_walk**

ros2 run ik_walk ik_walk


**camera**
1. ros2 launch ocam_ros2 ocam_ros.launch.py
2. ros2 launch robocup_vision25 robocup_vision25_launch.py


**motion**

ros2 launch motion_operator motion_operator.launch.py


**imu**

ros2 launch ebimu e2box_imu_9dofv4.launch.py


**master**

ros2 launch robocup_master25 robocup_master25.launch.py

**local**

ros2 run robocup_localization25 robocup_localization25
