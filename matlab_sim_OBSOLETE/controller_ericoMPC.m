% Use an MPC to find the correct system inputs F,M1,M2,M3
function [F,M] = controller_ericoMPC(t,MPCobj,state,des_state,sysparams,mpcparams)

    server_IP   = '127.0.0.1';   % this should be the IP of the MPC server
    server_port = 6004;          % port where the MPC server is listening
    sizeof_double = 8;
    num_iter = 200;
    n = 12;
    m = 4;
    
    % opening an UDP socket to the server
    socket_id = udp(server_IP, server_port);
    fopen(socket_id);
    

    % Arm Parameters
    l1x = sysparams.arm_mat(1,1);  l1y = sysparams.arm_mat(1,2);  
    l2x = sysparams.arm_mat(2,1);  l2y = sysparams.arm_mat(2,2);
    l3x = sysparams.arm_mat(3,1);  l3y = sysparams.arm_mat(3,2);
    l4x = sysparams.arm_mat(4,1);  l4y = sysparams.arm_mat(4,2);
    
    x = state;  % Current State
    r = des_state; % Reference to track
    
    x = x - r; % Offset x so that current state 
    
    fwrite(socket_id,x,'double');  % send state to the MPC server
	u = fread(socket_id,m*sizeof_double,'double'); % get the imput from MPC server
    F = u(1) + sysparams.mass*sysparams.grav;
    M = zeros(3,1);
    M(1) = u(2);
    M(2) = u(3);
    M(3) = u(4);
    
    
%     AH_i = evalin("base","AH_i"); % Index for action horizon
%     if AH_i == 1 % If at the beginning of the AH
%        [del_u,info] = mpcmoveAdaptive(MPCobj,x,Plant,[],ym,r,[]); % Find del_u and info
%        del_us = info.Uopt(2:mpcparams.AH,:)'; % Find further inputs for AH
%        assignin('base','del_us',del_us); % Save these inputs
%        if mpcparams.AH ~= 1
%         AH_i = AH_i + 1; % Increment AH index
%        end
%        assignin('base','AH_i',AH_i);
%     else
%         del_us = evalin('base','del_us'); % Get del_us
%         del_u = del_us(:,AH_i); % Get correct del_u
%         if AH_i == mpcparams.AH % If at AH, restart index
%            AH_i = 1; 
%         else % else, increment
%            AH_i = AH_i + 1; 
%         end
%     end
    
    assignin('base','x_mpc',x);
%     u = del_u + sysparams.mass*sysparams.grav/4/sysparams.kf * ones(4,1);
%     u_clamped = max(min(u, sysparams.maxF/4/sysparams.kf), sysparams.minF/4/sysparams.kf);

%     F = sysparams.kf*sum(u_clamped);
%     M = zeros(3,1);
%     M(1) = sysparams.kf*(l1x*u_clamped(1)...
%         + l2x*u_clamped(2)...
%         - l3x*u_clamped(3)...
%         - l4x*u_clamped(4));
%     M(2) = sysparams.kf*(-l1y*u_clamped(1)...
%         + l2y*u_clamped(2)...
%         + l3y*u_clamped(3)...
%         - l4y*u_clamped(4));
%     M(3) = sysparams.km*(u_clamped(1) - u_clamped(2)...
%         + u_clamped(3) - u_clamped(4));
    
    infos = evalin('base','infos');
    info.time = t;
    info.state = state;
    info.des_s = des_state;
%     info.ym = ym;
    infos.info = [infos.info;info];
    infos.F = [infos.F;F];
    infos.M = [infos.M;M'];
    assignin('base','infos',infos);
end