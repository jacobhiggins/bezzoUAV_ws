function timed_sim(obj,event)
    global sim_timer;
    global sysparams;
    global video;
    global noise_flag;
    global time;
    global tstep;
    global QP;
    global h_title;
    global h_fig;
    global x;
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
    global rpy;
    global key;
    global MPC_lag;
    
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
    
    
    
    dist = norm(state(1:3)-des_state(1:3),2); % Distance from UAV to current waypoint
    
    [Fnew, Mnew, dt_mpc] = controller_semaphore(state,des_state); % Get trpy for quadrotor
    if dt_mpc>tstep
        disp("WARNING");
        disp("Time for mpc is longer than sampling time");
    end
    
    % Record time + trpy for debugging purposes
    ts = [ts time+tstep];
    Fs = [Fs Fnew];
    Ms = [Ms Mnew];
    
    state_diff = state - mpc_state; % Take difference betwee current state and state used by mpc, for debugging
    states_mpc = [states_mpc mpc_state];
    state_diffs = [state_diffs state_diff];
    
    if MPC_lag < 0
        dt1 = dt_mpc;
    else
        dt1 = MPC_lag;
    end
    if(dt1>0.0001)
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, F, M, sysparams), [time,time+dt1], x{qn});
        x{qn}    = xsave(end, :)';
        t{qn} = tsave(end, :)';
        states = [states state];    
    end
    
    F = Fnew;
    M = Mnew;
    
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, F, M, sysparams), [time+dt1,time+tstep], x{qn}); % Change integration time to dt2 when testing
    
    x{qn}    = xsave(end, :)';
    t{qn} = tsave(end, :)';
    states = [states state];
    xtraj{qn}(iter,:,:) = x{qn}';
    ttraj{qn}(iter) = t{qn}';
    
    time = iter*tstep;
    
    % add noise
    if noise_flag
        x{qn} = add_noise(x{qn});
    end
    
    % Update quad plot
    QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + tstep);
    
    if video
        writeVideo(video_writer, getframe(h_fig));
    end
    
    [phi, theta, psi] = RotToRPY_ZXY(QP{qn}.rot');
    rpy = [rpy,[phi;theta;psi]]; % Record RPY to post-plotting
    set(h_title, 'String', sprintf('\niteration: %d, time: %4.2f\n\npress "q" to quit ', iter, time + tstep))
    
    % If 'q' is pressed on the keyboard, simulation stops and post-plotting
    % begins
    if(key=='q')
       sim_timer.stop(); 
    end

end