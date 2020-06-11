# Compilation of MPC controller

0. Clean old compilation stuff by
  ```
  make clean
  ```

1. Get the latest MPC library by
  ```
  make mpc_get_lib
  ```
  Upon successful completion, you should have got the following files in this directory: `mpc.h`, `dyn.h`, `mpc.o`, `dyn.o`. They are needed to compile your MPC controller. This step is needed only the first time or when you need to update the MPC library

## Local MPC execution

1. Make sure that USE_SERVER *is not* defined in mpc_ctrl.c. Then
  ```
  make
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

3. From any folder, this command runs roscore, the central node that is necessary for anything to run on ros:

```bash
roscore
```

4. Finally, run this command for the mpc:

```bash
rosrun enrico_mpc2_pk mpc_control2
```

--------------------------------------

Once you know the code works, you can try running the matlab sim with the mpc

0. If you have roscore running from the terminal, shut it down

1. From the matlab console, run:

```bash
rosinit
```

This is matlab's version of roscore. For some reason, we must use this and not roscore for our matlab ros node to talk with our C++ ros node.

2. In matlab, run the /bezzoUAV_ws/matlab_sim/runsim.m script

You should see the same simulator that I showed last Friday.
