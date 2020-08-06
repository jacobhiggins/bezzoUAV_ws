function desired_state = traj2(t, qn, waypoints, simparams)
    persistent i
    if isempty(i)
       i = 0; 
    end
    if t==0
       desired_state.pos = [0;0;0];
    else
       desired_state.pos = [simparams.mpcparams.Xref;...
           simparams.mpcparams.Yref;...
           simparams.mpcparams.Zref];
    end
    desired_state.euler = [0;0;0];
    desired_state.vel = [0;0;0];
    desired_state.pqr = [0;0;0];
end