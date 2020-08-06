% Input: current state and desired state (i.e. waypoint) from control loop
% Output:  
function [Fnew,Mnew,dt] = controller_semaphore(state,des_state)
    global mass;
    global grav;
    state = state - des_state;

    Fnew = 0;
    Mnew = zeros(3,1);
    
    [u, dt] = mpc_shm_ctrl_matlab(state);
    
    Fnew = u(1) + mass*grav;
    Mnew(1) = u(2);
    Mnew(2) = u(3);
    Mnew(3) = u(4);
    
%     dt = 0.01;
    
end