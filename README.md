# bezzoUAV_ws

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
