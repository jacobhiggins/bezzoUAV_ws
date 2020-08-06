function desired_state = traj3(waypoints, dist)
    persistent i
    if isempty(i)
       i = 0; 
    end
    if dist<0.05
       i =  mod(i,size(waypoints,1)) + 1;
    end
%     i = 2;
    if i==0
       desired_state.pos = [0;0;0];
       desired_state.euler = [0;0;0];
       desired_state.vel = [0;0;0];
       desired_state.pqr = [0;0;0];
    else
       desired_state.pos = waypoints(i,1:3)';
       desired_state.euler = waypoints(i,4:6)';
       desired_state.vel = waypoints(i,7:9)';
       desired_state.pqr = waypoints(i,10:12)';
    end
end