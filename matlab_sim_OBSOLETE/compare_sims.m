close all;
addpath("../../bezzoUAV_ws");
a = load("debug.csv");
csim_time = a(:,1);
csim_time1 = a(:,1) - a(1,1)*ones(length(csim_time),1) - 1.8*ones(length(csim_time),1);
csim_state = a(:,2:13);
csim_input = a(:,14:end);
load('matlabInputs.mat'); % time = t, thrust = Fs, moments = Ms

figure(1);
hold on;
plot(csim_time1,csim_input(:,1),"DisplayName","ROS Sim Thrust");
plot(ts,Fs,"DisplayName","Matlab Sim Thrust");
xlim([0,2]);
legend;
title("Comparing Matlab and ROS Sim MPC Commands");

figure(2);
hold on;
yyaxis right;
plot(csim_time,csim_input(:,1),"DisplayName","ROS Sim Thrust");
ylabel("Force");
yyaxis left;
plot(csim_time,csim_state(:,3),"DisplayName","Z");
ylabel("Meters");
xlabel("Time (s)");
xlim([4,6.5]);
title("ROS Sim Pos vs. Thrust");

figure(3);
hold on;
yyaxis right;
plot(ts,Fs,"DisplayName","Matlab Sim Thrust");
ylabel("Force");
yyaxis left;
plot(ts(1:end-1),states(3,:),"DisplayName","Z");
ylabel("Meters");
xlabel("Time (s)");
title("Matlab Sim Pos and Thrust");
