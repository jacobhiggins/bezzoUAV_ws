function [xtraj, ttraj, terminate_cond] = test_trajectory_timed(start, wypts)

% Get nquad
nquad = 1;

% Set video to true to record
% Video 
global video;
video = true;
vidname = "MPC_12order.avi";

% Quadrotor model
% Set model (e.g. mass, moment of inertia) of quadrotor
global sysparams;
% sysparams = nanoplus();
sysparams = iris();

% Set mass and gravity for simulation
global mass;
global grav;
mass = sysparams.mass;
grav = sysparams.grav;


%% **************************** FIGURES *****************************
% Environment figure
global h_fig;
h_fig = figure('Name', 'Environment');
h_3d = gca;
drawnow;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
axis([-0.1 1.5 -0.1 1.5 -0.1 1.5]);
axis equal;
view(3);
% axis equal;
grid on;
quadcolors = lines(nquad);
set(gcf,'Renderer','OpenGL')
set(h_fig, 'KeyPressFcn', @(h_fig, evt)MyKeyPress_Cb(evt.Key));

global video_writer;

if video
    video_writer = VideoWriter(vidname, 'Motion JPEG AVI');
    open(video_writer);
end
%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
% Maximum time that the quadrotor is allowed to fly
time_tol = 50;          % maximum simulation time
starttime = 0;          % start of simulation in seconds
global ctrl_time;
global tstep;
tstep     = ctrl_time;
global time;
time      = starttime;  % current time
max_iter  = time_tol / (tstep);      % max iteration
global x0;
global xtraj;
global ttraj;
for qn = 1:nquad
    % Get start and stop position
    x0{qn}    = init_state( start );
    xtraj{qn} = zeros(max_iter, length(x0{qn}));
    ttraj{qn} = zeros(max_iter, 1);
end
global x;
x = x0;        % state
% AH_i = 1;
% assignin("base","AH_i",AH_i);
global ts;
global Fs;
global Ms;
ts = [0];
Fs = [0];
Ms = [0;0;0];
states = [];
global F;
global M;
F = mass*grav;
M = [0;0;0];
global mpc_state;
global rpy;
global state_diffs;
global states_mpc;
mpc_state = zeros(12,1);
rpy = zeros(3,1);
state_diffs = [];
states_mpc = [];


%% ************************* RUN SIMULATION *************************
global key;
global QP;
global states;
global h_title;
global iter;
global waypoints;
waypoints = wypts;
iter = 1;
key = -1;
fprintf('Simulation Running....\n')
dist = 10;

QP{qn} = QuadPlot(qn, x0{qn}, sysparams.arm_mat, 0.04, quadcolors(qn,:), max_iter, h_3d);
desired_state = traj3(wypts,dist);
QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
h_title = title(sprintf('\niteration: %d, time: %4.2f\n\npress "q" to quit ', 1, time));

global sim_timer;
global err 
err = false;
sim_timer = timer;
sim_timer.Period = tstep;
sim_timer.TasksToExecute = int16(time_tol/tstep); % Max time = 100 seconds
sim_timer.ExecutionMode = 'fixedRate'; % periodic execution
sim_timer.TimerFcn = {@timed_sim}; % timer handler function
sim_timer.ErrorFcn = {@errFunction}; % If error in TimerFcn, end code
sim_timer.start(); % Start timed execuation of simulation

wait(sim_timer); % This stops code until sim is done

if err
   return; 
end

key = -1;
%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter,:);
    ttraj{qn} = ttraj{qn}(1:iter);
end

% Plot the saved position and velocity of each robot
for qn = 1:nquad
    % Truncate saved variables
    QP{qn}.TruncateHist();
    % Plot position for each quad
    h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
    plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
    plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
    % Plot orientation for each quad
    h_ori{qn} = figure('Name', ['Quad ' num2str(qn) ' : euler']);
    plot_state(h_ori{qn}, rpy, QP{qn}.time_hist, 'euler', 'vic');
    % Plot velocity for each quad
    h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
    plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
    plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
end

if video
  close(video_writer);
end

states_matlab = states;
save("matlabInputs.mat",'Fs','Ms','ts','states');
save("state_diffs.mat","state_diffs","states_matlab","states_mpc","tstep");

assignin('base','Ms',Ms);
assignin('base','t',QP{qn}.time_hist);

end
