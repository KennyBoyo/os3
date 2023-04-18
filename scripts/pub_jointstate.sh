# BASH SCRIPT TO CONTINUOUSLY SEND FORCE DATA AT A RATE EQUAL TO THE PUBLISH RATE OF THE FRANKA PANDA

source ~/.bashrc

OS3_FORCE_TOPIC_PROXY=/problem_topic
OS3_FORCE_TYPE=sensor_msgs/JointState
OS3_PUB_FREQ=20

FORCE_TIME=-1.0; # dummy time to be adapted by isosim

rostopic pub -r $OS3_PUB_FREQ $OS3_FORCE_TOPIC_PROXY $OS3_FORCE_TYPE "{header: {seq: 0, stamp: {secs: 1, nsecs: 10}, frame_id: \"panda_K\"}, name: ["Flexion", "Abduction", "Rotation", "Elbow"], position: [1.57, 1.57, 1.57, 1.57], velocity: [0, 0, 0, 0], effort: [1, 1, 1, 1]}"
