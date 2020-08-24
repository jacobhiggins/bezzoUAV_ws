# MPC controller for ROS
This repo contains the code for controlling a UAV by MPC.

## Cloning the Repo

Note: This repo contains submodules of other repos. In order to clone all necessary files, please use:
```
git clone git@github.com:jacobhiggins/bezzoUAV_ws.git --recurse-submodules
```

## Preliminaries

Before running the MPC controller, it is necessary to download and compile the MPC library. This step is necessary only the first time of the installation. It can be made by:

1. Clean old compilation stuff by launching the following command from the root project directory
  ```
  make mpc_clean
  ```
This may be necessary to update the MPC library

2. Get the latest MPC library by
  ```
  make mpc_get_lib
  ```

Upon successful completion, you should have got the following files in the `mpc` directory: `mpc.h`, `dyn.h`, `mpc.o`, `dyn.o`. They are needed to compile your MPC controller.

## MPC controller (`mpc_shm_ctrl`)

This is the executable running the local MPC controller. Such a controller interacts with the system to be controlled through a shared memory. The `struct shared_data` declared in `mpc/mpc_interface.h`


2. Compile the MPC controller using the shared memory and used by Matlab, by:
  ```
  make mpc_shm_ctrl
  ```

## Local MPC execution

1. Make sure that USE_SERVER *is not* defined in mpc_ctrl.c. Then
  ```
  make
  ```

## Compilation of a small tracing tool

1. From the home directory of the project, launch
  ```
  make trace_proc
  ```


## Total off-loading to MPC server

1. Make sure that USE_SERVER *is* defined in mpc_ctrl.c. Also, the C macros
SOLVER_IP and SOLVER_PORT should be the IP address and the port where the server is listening. Default values are
  ```
  #define SOLVER_IP "127.0.0.1"
  #define SOLVER_PORT 6001
  ```
  Then
  ```
  make
  ```

1. Before launching the rosrun machinery, start-up the server by launching in a dedicated server terminal. If the server is launched at localhost, just launch
  ```
  ./launch_MPC_server
  ```
  from the project home directory.

2. Now run rosrun enrico_mpc2_pk mpc_control2


# Instructions to run ROS

## Preliminaries

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
## Running the simulation

The simulation can be launched from the following bash command:

```bash
roslaunch iris_simulator_pk iris_mpc.launch
```
This launches both the simulator and MPC controller.

Note that you do not need to visualize the simulation with rviz to simulation motion.

Using the keyboard window, you can give the simulation the following commands:
3 - Lift off ground
1 - Waypoints in the shape of house

