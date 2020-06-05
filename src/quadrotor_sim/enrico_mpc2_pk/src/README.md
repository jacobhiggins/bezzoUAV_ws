# Compilation of MPC controller

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


