%% Fake system to test

%% Linearize the system

%% Discretize the linearized system with Ts

%% Write the JSON file with the sampling time Ts

global A B x
A = [0.5 -0.5; 0.5 0.5];
B = [1; 1];
Ts = 1;
x = [10; 10];


%% Creating a timer with period t.Period
t = timer;
t.Period = Ts;
t.ExecutionMode = 'fixedRate'; % periodic execution
t.TimerFcn = {@my_callback_fcn}; % timer handler function

%% Let's go
t.start();

%% Stop the simulation by t.stop


