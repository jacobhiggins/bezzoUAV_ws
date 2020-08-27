# ROS simulation of MPC controller
This repo contains the code for controlling a UAV by MPC.

## Cloning the Repo

Note: This repo contains submodules of other repos. In order to clone all necessary files, please use:
```
git clone git@github.com:jacobhiggins/bezzoUAV_ws.git --recurse-submodules
```

## Launching the MPC controller

Before running the ROS simulator, it is necessary to compile the MPC library. To launch the MPC controller, follows the next steps:

1. Open a dedicated terminal

1. Visit the directory of the MPC module, by
  ```
cd src/quadrotor_sim/mpc_offloading/mpc_submodule/
  ```

1. Compile the latest version of the controller by
  ```
make mpc_ctrl
  ```
1. Have a json file describing the MPC problem ready

1. Launch the MPC controller by
  ```
./mpc_ctrl <json-model-filename>
  ```


## Compiling the ROS code

To run the code:

0. If this is the first time the directory is being compiled, first compile the quadrotor_msgs package:

```bash
catkin_make --pkg quadrotor_msgs
```

This is a dependency needed for other files.

1. From the workspace parent directory, run:

```bash
catkin_make
```

This compiles the code that ros can run.

2. Still from the parent directory, run:

```bash
source ./devel/setup.bash
```
This step can usually be skipped by adding this line to the .bashrc file.

## Running the ROS simulation

The simulation can be launched from the following bash command:

```bash
roslaunch iris_simulator_pk iris_mpc.launch
```
This launches both the simulator and MPC controller.

Note that you do not need to visualize the simulation with rviz to simulation motion.

Using the keyboard window, you can give the simulation the following commands:
3 - Lift off ground
1 - Waypoints in the shape of house

