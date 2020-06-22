# Communication via shared memory (TEST)

The file `common.test` contains the data structure declarations.

1. Compile the (fake) MPC controller by
  ```
  make mpc_shm_ctrl
  ```

1. compile the MEX file by
  ```
  mex -O -v mpc_shm_ctrl_matlab_test.c
  ```

1. Open a terminal and launch
  ```
  ./mpc_shm_ctrl_test
  ```
  This will create the shared memory and the MPC controller listening

1. Open Matlab and launch the script
  ```
  test_periodic
  ```

1. The shared memory will be canceled when it is pressed Ctrl-C on the terminal with `./mpc_shm_ctrl_test` running
