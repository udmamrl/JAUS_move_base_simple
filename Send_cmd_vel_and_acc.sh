#!/bin/bash
while true
do
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "odom" }, pose: { position: { x: -4.5, y: 5.5 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
rostopic pub -1 /move_base_simple/max_acceleration std_msgs/Float32 '{ data: 10 }'
sleep 13

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "odom" }, pose: { position: { x: 4.5, y: 5.5 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
rostopic pub -1 /move_base_simple/max_acceleration std_msgs/Float32 '{ data: 10 }'
sleep 8

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "odom" }, pose: { position: { x: -4.5, y: -5.5 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
rostopic pub -1 /move_base_simple/max_acceleration std_msgs/Float32 '{ data: 10 }'
sleep 13


rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id: "odom" }, pose: { position: { x: 4.5, y: -5.5 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
rostopic pub -1 /move_base_simple/max_acceleration std_msgs/Float32 '{ data: 2 }'
sleep 8

done
