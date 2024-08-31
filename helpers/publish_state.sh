screen ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro ~/ar4_ros_driver/ar_description/urdf/ar.urdf.xacro ar_model:=ar4_mk3)"
screen ros2 run joint_state_publisher_gui joint_state_publisher_gui

