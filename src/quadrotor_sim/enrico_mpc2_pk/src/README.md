# Fully local execution

1. Get the latest MPC library by
```
make mpc_get_lib
```
...Upon successful completion, you should have got the following files in this directory: `mpc.h`, `dyn.h`, `mpc.o`, `dyn.o`. They are needed to compile your MPC controller. This step is needed only the first time or when you need to update the MPC library

1. Compiling your MPC controller by
```
make
```
