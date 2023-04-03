# BASH SCRIPT TO CONTINUOUSLY SEND FORCE DATA AT A RATE EQUAL TO THE PUBLISH RATE OF THE FRANKA PANDA

source ~/.bashrc

ISOSIM_FORCE_TOPIC=/ROSforceOutput
ISOSIM_FORCE_TYPE=franka_panda_controller_swc/ForceOutput
ISOSIM_PUB_FREQ=20

FORCE_TIME=-1.0; # dummy time to be adapted by isosim

rostopic pub -r $ISOSIM_PUB_FREQ  $ISOSIM_FORCE_TOPIC $ISOSIM_FORCE_TYPE "{force: {x: 100.0, y: 0.0, z: -10.0}, time: $FORCE_TIME }"
