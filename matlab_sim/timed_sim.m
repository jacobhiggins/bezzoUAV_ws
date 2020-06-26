function timed_sim(obj,event)
    global sim_timer;
    global sysparams;
    global video;
    global noise_flag;
    global time;
    global tstep;
    global nstep;
    cstep = nstep*tstep;
    global QP;
    global h_title;
    global h_fig;
    global x;
    global x0;
    global xtraj;
    global ttraj;
    global F;
    global M;
    global mpc_state;
    global state_diffs;
    global states_mpc;
    global states;
    global Fs;
    global Ms;
    global ts;
    global video_writer;
    global iter;
    global waypoints;
    persistent rpy;
    if isempty(rpy)
       rpy = zeros(3,1); 
    end
    persistent dist;
    if isempty(dist)
       dist = 10; 
    end
    
    
    iter = iter + 1;
    qn = 1;
    
    desired_state = traj3( waypoints, dist);
    
    des_state = [desired_state.pos;desired_state.euler;desired_state.vel;desired_state.pqr]; % For MPC controller
    
    
    % Use MPC to calculate control input
    current_state = x{qn};
    
    
    % For MPC through Matlab
    % Convert quaternion to rpy angles, form correct state vector
    qw = current_state(7);
    qx = current_state(8);
    qy = current_state(9);
    qz = current_state(10);
    R = QuatToRot([qw,qx,qy,qz]);
    [phi,theta,psi] = RotToRPY_ZXY(R);
    state = [current_state(1);... % x
        current_state(2);... % y
        current_state(3);... % z
        phi;... % roll
        theta;... % pitch
        psi;... % yaw
        current_state(4);... % xdot
        current_state(5);... % ydot
        current_state(6);... % zdot
        current_state(11);... % p
        current_state(12);... % q
        current_state(13);]; % r
    
    
    % Publish current state to ROS topic for enrico MPC
    
    dist = norm(state(1:3)-des_state(1:3),2);
    
    %             [F, M] = controllerMPC(time,MPCobj,state,des_state,sysparams,mpcparams);
    %             [F, M] = controller_ericoMPC(time,MPCobj,state,des_state,sysparams,mpcparams);
    
    [Fnew, Mnew, dt_mpc] = controller_semaphore(state,des_state); % Get trpy for quadrotor
    if dt_mpc>tstep
        disp("WARNING");
        disp("Time for mpc is longer than sampling time");
    end
    
    
    ts = [ts time+tstep];
    Fs = [Fs Fnew];
    Ms = [Ms Mnew];
    %             disp(F/(sysparams.mass*sysparams.grav));
    
    state_diff = state - mpc_state; % Take difference betwee current state and state used by mpc, for debugging
    states_mpc = [states_mpc mpc_state];
    state_diffs = [state_diffs state_diff];
    
    %         dt1 = dt_mpc;
    %         dt2 = tstep - dt_mpc;
    %         [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, F, M, sysparams), [time,time+dt1], x{qn})
    %         x{qn}    = xsave(end, :)';
    %         t{qn} = tsave(end, :)';
    %         states = [states state];
    %         xtraj{qn}((iter-1)*nstep+i,:) = x{qn}';
    %         ttraj{qn}((iter-1)*nstep+i) = t{qn}';
    
    F = Fnew;
    M = Mnew;
    
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, F, M, sysparams), [time,time+tstep], x{qn}); % Change integration time to dt2 when testing
    
    x{qn}    = xsave(end, :)';
    t{qn} = tsave(end, :)';
    states = [states state];
    xtraj{qn}(iter*nstep,:,:) = x{qn}';
    ttraj{qn}(iter*nstep) = t{qn}';
    
    
    
    
    time = iter*tstep;
    
    % add noise
    if noise_flag
        x{qn} = add_noise(x{qn});
    end
    
    % Save to traj
    %         xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    %         ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    % Update quad plot
    %         desired_state = trajhandle(time + cstep, qn, simparams);
    QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
    
    if video && abs(mod(time,0.1) - tstep)<0.0001
        writeVideo(video_writer, getframe(h_fig));
    end
    [phi, theta, psi] = RotToRPY_ZXY(QP{qn}.rot');
    rpy = [rpy,[phi;theta;psi]];
    set(h_title, 'String', sprintf('\niteration: %d, time: %4.2f\n\npress "q" to quit ', iter, time + cstep))
    %     time = time + cstep; % Update simulation time
    
%     if(key=='q')
%        sim_timer.stop(); 
%     end

end