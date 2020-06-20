function my_callback_fcn(obj, event)

global x A B

% Invoke the MPC
[u, time] = mpc_shm_ctrl_matlab_test(x);

%Printing the time taken by MPC
disp(time)

% simulate the dynamics
x = A*x+B*u


