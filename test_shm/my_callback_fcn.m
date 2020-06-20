function my_callback_fcn(obj, event)

global x A B

% Invoke the MPC
u = mpc_shm_ctrl_matlab_test(x);

% simulate the dynamics
x = A*x+B*u


