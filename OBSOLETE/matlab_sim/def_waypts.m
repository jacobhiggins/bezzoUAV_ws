function waypts = def_waypts()
vel = 0;
side = 1;
h = 1;
% start points used to simulate takeoff
start_pt = [0 0 h 0 0 0 vel 0 0 0 0 0];

% defined waypts here
%**************** define waypts ***************************** 
waypts = [side 0 h 0 0 0 0 vel 0 0 0 0;
          side side h 0 0 0 -vel 0 0 0 0 0;
          0 side h 0 0 0 0 -vel 0 0 0 0]; % square
% waypts = [1 0 0 0 0 0 0 0 0 0 0 0];
%     1 0 1 0 0 0 0 0 0 0 0 0];

      
%******************* end ************************************

waypts = [start_pt;
          waypts];

end

