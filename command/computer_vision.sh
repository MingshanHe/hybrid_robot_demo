gnome-terminal -- bash  -c "roslaunch realsense2_camera rs_rgbd.launch"

gnome-terminal -- bash  -c "roslaunch aruco_ros single.launch"

gnome-terminal -- bash  -c "roslaunch realsense2_camera aruco_tf_node"