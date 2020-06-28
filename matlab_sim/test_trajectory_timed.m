function [xtraj, ttraj, terminate_cond] = test_trajectory_timed(start, stop, map, path, MPCobj, noise_flag, simparams, wypts)
% TEST_TRAJECTORY simulates the robot from START to STOP following a PATH
% that's been planned for MAP.
% start - a 3d vector or a cell contains multiple 3d vectors
% stop  - a 3d vector or a cell contains multiple 3d vectors
% map   - map generated by your load_map
% path  - n x 3 matrix path planned by your dijkstra algorithm
% trajectory generator handles
trajhandle    = @traj3;

% Make cell
if ~iscell(start), start = {start}; end
if ~iscell(stop),  stop  = {stop}; end
if ~iscell(path),  path  = {path} ;end

% Get nquad
nquad = length(start);

global video;
video = true;

% Make column vector
for qn = 1:nquad
    start{qn} = start{qn}(:);
    stop{qn} = stop{qn}(:);
end

% Quadrotor model
% sysparams = nanoplus();
global sysparams;
sysparams = iris();
mpcparams = simparams.mpcparams;
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
    video_writer = VideoWriter("MPC_3D_test.avi", 'Motion JPEG AVI');
    open(video_writer);
end
%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
% Maximum time that the quadrotor is allowed to fly
time_tol = 50;          % maximum simulation time
starttime = 0;          % start of simulation in seconds
global tstep;
global nstep;
tstep     = mpcparams.Ts;       % this determines the time step at which the solution is given
nstep     = 1;
cstep     = nstep*tstep;       % image capture time interval
global time;
time      = starttime;  % current time
max_iter  = time_tol / (nstep*tstep);      % max iteration
global x0;
global xtraj;
global ttraj;
for qn = 1:nquad
    % Get start and stop position
    x0{qn}    = init_state( start{qn} );
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
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
global state_diffs;
global states_mpc;
mpc_state = zeros(12,1);
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
sim_timer = timer;
sim_timer.Period = tstep;
% sim_timer.TasksToExecute = int16(100/tstep);
sim_timer.ExecutionMode = 'fixedRate'; % periodic execution
sim_timer.TimerFcn = {@timed_sim}; % timer handler function

sim_timer.start();

wait(sim_timer);


% Main loop
% while key ~= 'q'
%     iter = iter + 1;
% 
%     % Iterate over each quad
%     for qn = 1:nquad
%         % Initialize quad plot
%         if iter == 1
% %             QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
%             QP{qn} = QuadPlot(qn, x0{qn}, sysparams.arm_mat, 0.04, quadcolors(qn,:), max_iter, h_3d);
%             desired_state = traj3(waypoints,dist);
%             QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
%             h_title = title(sprintf('\niteration: %d, time: %4.2f\n\npress "q" to quit ', iter, time));
%             
%             % Put in obstacle
% %             plotcube([0.25 0.25 0.25],[0.5 0.5 0.5],.8,[1 0 0]);
%         else
%            desired_state = traj3( waypoints, dist);
%         end
%         
%         des_state = [desired_state.pos;desired_state.euler;desired_state.vel;desired_state.pqr]; % For MPC controller
%         
% 
%         % Use MPC to calculate control input
%         current_state = x{qn};
%         
%         
%         % For MPC through Matlab
%         % Convert quaternion to rpy angles, form correct state vector
%         qw = current_state(7);
%         qx = current_state(8);
%         qy = current_state(9);
%         qz = current_state(10);
%         R = QuatToRot([qw,qx,qy,qz]);
%         [phi,theta,psi] = RotToRPY_ZXY(R);
%         state = [current_state(1);... % x
%             current_state(2);... % y
%             current_state(3);... % z
%             phi;... % roll
%             theta;... % pitch
%             psi;... % yaw
%             current_state(4);... % xdot
%             current_state(5);... % ydot
%             current_state(6);... % zdot
%             current_state(11);... % p
%             current_state(12);... % q
%             current_state(13);]; % r
%         
%         
%         % Publish current state to ROS topic for enrico MPC
% 
%         dist = norm(state(1:3)-des_state(1:3),2);
% 
%         %             [F, M] = controllerMPC(time,MPCobj,state,des_state,sysparams,mpcparams);
%         %             [F, M] = controller_ericoMPC(time,MPCobj,state,des_state,sysparams,mpcparams);
%         
%         [Fnew, Mnew, dt_mpc] = controller_semaphore(state,des_state); % Get trpy for quadrotor
%         if dt_mpc>tstep
%            disp("WARNING");
%            disp("Time for mpc is longer than sampling time");
%         end
%         
%         
%         ts = [ts time+tstep];
%         Fs = [Fs Fnew];
%         Ms = [Ms Mnew];
%         %             disp(F/(sysparams.mass*sysparams.grav));
%         
%         state_diff = state - mpc_state; % Take difference betwee current state and state used by mpc, for debugging
%         states_mpc = [states_mpc mpc_state];
%         state_diffs = [state_diffs state_diff];
%         
% %         dt1 = dt_mpc;
% %         dt2 = tstep - dt_mpc;
% %         [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, F, M, sysparams), [time,time+dt1], x{qn})
% %         x{qn}    = xsave(end, :)';
% %         t{qn} = tsave(end, :)';
% %         states = [states state];
% %         xtraj{qn}((iter-1)*nstep+i,:) = x{qn}';
% %         ttraj{qn}((iter-1)*nstep+i) = t{qn}';
%         
%         F = Fnew;
%         M = Mnew;
%         
%         [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, F, M, sysparams), [time,time+tstep], x{qn}); % Change integration time to dt2 when testing
%         
%         x{qn}    = xsave(end, :)';
%         t{qn} = tsave(end, :)';
%         states = [states state];
%         xtraj{qn}((iter-1)*nstep+i,:) = x{qn}';
%         ttraj{qn}((iter-1)*nstep+i) = t{qn}';
%         
%         
%         
%         
%         time = cstep*(iter-1) + tstep*i;
% 
%         % add noise
%         if noise_flag
%             x{qn} = add_noise(x{qn});
%         end
%        
%         % Save to traj
% %         xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
% %         ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
% 
%         % Update quad plot
% %         desired_state = trajhandle(time + cstep, qn, simparams);
%         QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
%         
%         if video && abs(mod(time,0.1) - tstep)<0.0001
%             writeVideo(video_writer, getframe(h_fig));
%         end
%     end
%     [phi, theta, psi] = RotToRPY_ZXY(QP{qn}.rot');
%     rpy = [rpy,[phi;theta;psi]];
%     set(h_title, 'String', sprintf('\niteration: %d, time: %4.2f\n\npress "q" to quit ', iter, time + cstep))
% 
% end
key = -1;
%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
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

% figure(5);
% subplot(2,1,1);
% plot(ts,Fs,"LineWidth",2);
% title("Thrust");
% subplot(2,1,2);
% hold on;
% plot(ts,Ms(1,:),"LineWidth",2,"DisplayName","Roll Moment");
% plot(ts,Ms(2,:),"LineWidth",2,"DisplayName","Pitch Moment");
% plot(ts,Ms(3,:),"LineWidth",2,"DisplayName","Yaw Moment");
% legend;

states_matlab = states;
save("matlabInputs.mat",'Fs','Ms','ts','states');
save("state_diffs.mat","state_diffs","states_matlab","states_mpc","tstep");

assignin('base','Ms',Ms);
assignin('base','t',QP{qn}.time_hist);

end
