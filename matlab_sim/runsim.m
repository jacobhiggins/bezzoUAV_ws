% 12 order quadrotor simulator
% Matlab 

close all;
clear all;
clc;
addpath('utils');
addpath('../test_shm');
sysparams = iris(); % physical characteristics of the system

% Noise flag
global noise_flag;
noise_flag        = false;

% Timing parameters
global ctrl_time;
global MPC_lag;
ctrl_time = 0.1; % Time between accessing the MPC controller
MPC_lag = 0.02; % Fixed lag time for MPC computation, value -1 means use actual computation time

%% Loading Waypts
disp('Loading Waypts ...');
waypts = def_waypts(); % Define waypoints for the UAV to travel to cyclicly
start = [0;0;0;0;0;0;0;0;0;0;0;0;0]; % Starting position of UAV

%% Run trajectory
trajectory = test_trajectory_timed(start, waypts); % Use timer to execute sim loop at fixed frequency
