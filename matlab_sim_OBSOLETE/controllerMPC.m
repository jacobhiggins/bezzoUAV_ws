% Use an MPC to find the correct system inputs F,M1,M2,M3
function [F,M] = controllerMPC(t,MPCobj,state,des_state,sysparams,mpcparams)

    % Arm Parameters
    l1x = sysparams.arm_mat(1,1);  l1y = sysparams.arm_mat(1,2);  
    l2x = sysparams.arm_mat(2,1);  l2y = sysparams.arm_mat(2,2);
    l3x = sysparams.arm_mat(3,1);  l3y = sysparams.arm_mat(3,2);
    l4x = sysparams.arm_mat(4,1);  l4y = sysparams.arm_mat(4,2);

    [Plant] = linModel(state,mpcparams,sysparams); % Find linearized state space around nominal point
    
    
    x = mpcstate(MPCobj); % Make an MPC state for the mpcmove function
    x.Plant = state;
    
    ym = state; % Measured output, assume full state is known
    
    r = des_state; % Reference to track
    
    AH_i = evalin("base","AH_i"); % Index for action horizon
    if AH_i == 1 % If at the beginning of the AH
       [del_u,info] = mpcmoveAdaptive(MPCobj,x,Plant,[],ym,r,[]); % Find del_u and info
       del_us = info.Uopt(2:mpcparams.AH,:)'; % Find further inputs for AH
       assignin('base','del_us',del_us); % Save these inputs
       if mpcparams.AH ~= 1
        AH_i = AH_i + 1; % Increment AH index
       end
       assignin('base','AH_i',AH_i);
    else
        del_us = evalin('base','del_us'); % Get del_us
        del_u = del_us(:,AH_i); % Get correct del_u
        if AH_i == mpcparams.AH % If at AH, restart index
           AH_i = 1; 
        else % else, increment
           AH_i = AH_i + 1; 
        end
    end
    
    assignin('base','x_mpc',x);
    u = del_u + sysparams.mass*sysparams.grav/4/sysparams.kf * ones(4,1);
    u_clamped = max(min(u, sysparams.maxF/4/sysparams.kf), sysparams.minF/4/sysparams.kf);

    F = sysparams.kf*sum(u_clamped);
    M = zeros(3,1);
    M(1) = sysparams.kf*(l1x*u_clamped(1)...
        + l2x*u_clamped(2)...
        - l3x*u_clamped(3)...
        - l4x*u_clamped(4));
    M(2) = sysparams.kf*(-l1y*u_clamped(1)...
        + l2y*u_clamped(2)...
        + l3y*u_clamped(3)...
        - l4y*u_clamped(4));
    M(3) = sysparams.km*(u_clamped(1) - u_clamped(2)...
        + u_clamped(3) - u_clamped(4));
    
    infos = evalin('base','infos');
    info.time = t;
    info.state = state;
    info.des_s = des_state;
    info.ym = ym;
    infos.info = [infos.info;info];
    infos.F = [infos.F;F];
    infos.M = [infos.M;M'];
    assignin('base','infos',infos);
end