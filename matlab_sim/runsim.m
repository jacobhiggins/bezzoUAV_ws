% 12 order quadrotor simulator
% Matlab 

close all;
clear all;
clc;
addpath('utils');
addpath('../test_shm');
sysparams = iris(); % physical characteristics of the system


varargin = {"Ts",0.1,...
    "PH",20,...
    "CH",2,...
    "Xref",1,...
    "Yref",1,...
    "Zref",1};
simparams = sim_params(varargin);

% flag
noise_flag        = false;

%% Loading Waypts
disp('Loading Waypts ...');
waypts = def_waypts();

map = {};
start = {[0;0;0;0;0;0;0;0;0;0;0;0;0]};
stop  = {waypts(end,:)};
path{1} = waypts;

%% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path);

%% Create MPC obj
MPCobj = [];
%MPCobj = makeMPCobj(sysparams,simparams);
% warning('off','MPC:computation:HessianSingular'); % Turn off warning from mpcstate in controllerMPC
% old_status = mpcverbosity('off'); % Turn off mpc controller verbosity
%% Run trajectory
infos = struct();
infos.info = [];
infos.F = [];
infos.M = [];
% trajectory = test_trajectory(start, stop, map, path, MPCobj, noise_flag, simparams, waypts); % with visualization
trajectory = test_trajectory2(start, stop, map, path, MPCobj, noise_flag, simparams, waypts); % Exchange data with mpc via semaphore
