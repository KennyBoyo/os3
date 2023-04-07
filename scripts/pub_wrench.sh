# BASH SCRIPT TO CONTINUOUSLY SEND FORCE DATA AT A RATE EQUAL TO THE PUBLISH RATE OF THE FRANKA PANDA

source ~/.bashrc

OS3_FORCE_TOPIC_PROXY=/franka_state_controller/F_ext
OS3_FORCE_TYPE=geometry_msgs/WrenchStamped
OS3_PUB_FREQ=20

FORCE_TIME=-1.0; # dummy time to be adapted by isosim

rostopic pub -r $OS3_PUB_FREQ $OS3_FORCE_TOPIC_PROXY $OS3_FORCE_TYPE "{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \"panda_K\"}, wrench: {force: {x: 10000.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"
