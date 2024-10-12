ros2 topic pub -r $3 /specified/pose geometry_msgs/msg/PoseStamped "pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}"
