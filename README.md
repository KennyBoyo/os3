# os3
Opensim Step Solver

Opensim Step Solver was built off of Isosim by Joshua Rolls: https://github.com/apatakiparadise/isosim

This repository modifies the code to operate in Opensim 4.3 and also perform inverse dynamics on a patient's right arm by passing in an end effector force (on the wrist of the patient) and joint angles of the arm. This repository is structured to 

# Getting Started
Clone this repository with all its submodules and follow the steps on [here](https://github.com/opensim-org/opensim-core/wiki/Build-Instructions#build-instructions-1) to install OpensSim or if you're on Linux, simply run the following command:

```
./opensim_build_script.sh
```

This should automatically install OpenSim and its relevant dependencies. Next, create a build directory and compile the C++ code:
```
mkdir build
cd build
cmake ..
make -j$(nproc)
```


The code to process the ROS data has been split into 3 different files - OS3, OS3Engine, and OS3ROS. OS3ROS serves as the interface between OS3 and ROS and required 
