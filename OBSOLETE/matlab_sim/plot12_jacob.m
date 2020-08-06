close all;
executable = './test_sys';
json_model = 'json/uav12_drag.json';
vars_name  = 'var';
num_steps  = 50;

launched_str = sprintf('%s %s %s %d',executable,json_model,vars_name, num_steps);
system(launched_str);
load(strcat(vars_name,'_X'), '-ascii');
load(strcat(vars_name,'_U'), '-ascii');
load(strcat(vars_name,'_steps'), '-ascii');
load(strcat(vars_name,'_time'), '-ascii');

figx = figure(1);
plot(0.1*(0:num_steps),var_X(1,:),'DisplayName','X position',"LineWidth",2);
xlabel("Time (s)");
ylabel("Distance (m)");
title("X Position");
legend;
figy = figure(2);
plot(0.1*(0:num_steps),var_X(2,:),'DisplayName','Y position',"LineWidth",2);
xlabel("Time (s)");
ylabel("Distance (m)");
title("Y Position");
legend;
figz = figure(3);
plot(0.1*(0:num_steps),var_X(3,:),'DisplayName','Z position',"LineWidth",2);
xlabel("Time (s)");
ylabel("Distance (m)");
title("Z Position");
legend;
figxyz = figure(4);
plot3(var_X(1,:),var_X(2,:),var_X(3,:),"LineWidth",2);
title("Trajectory");
xlabel("X Pos (m)");
ylabel("Y Pos (m)");
zlabel("Z Pos (m)");
axis equal;
figinputs = figure(5);
hold on;
yyaxis left;
plot(0.1*(0:(num_steps-1)),var_U(1,:),'DisplayName','Thrust',"LineWidth",2);
yyaxis right;
plot(0.1*(0:(num_steps-1)),var_U(2,:),'DisplayName',"Roll Moment","LineWidth",2);
plot(0.1*(0:(num_steps-1)),var_U(3,:),'DisplayName',"Pitch Moment","LineWidth",2);
title("Inputs");
legend;
figrp = figure(6);
yyaxis left;
plot(0.1*(0:num_steps),var_X(4,:)*180/pi,'DisplayName','Roll',"LineWidth",2);
xlabel("Time (s)");
ylabel("Angle (deg)");
yyaxis right;
plot(0.1*(0:num_steps),var_X(5,:)*180/pi,'DisplayName','Pitch',"LineWidth",2);
xlabel("Time (s)");
ylabel("Angle (deg)");
title("Roll and Pitch");
legend;
% figpitch = figure(7);
% plot(0.1*(0:18),var_X(5,:),'DisplayName','Pitch',"LineWidth",2);
xlabel("Time (s)");
%%
% 
% # ITEM1
% 
% # <http://www.mathworks.com ITEM1
% 
%   for x = 1:10
% 
% # ITEM1
% # ITEM2
% 
%       disp(x)
%   end
% 
% # ITEM2>
% 
% # ITEM2
% 
% ylabel("Angle (deg)");
% title("Pitch");
% legend;

saveas(figx,"./pics/xpos.png");
saveas(figy,"./pics/ypos.png");
saveas(figz,"./pics/zpos.png");
saveas(figxyz,"./pics/traj.png");
saveas(figinputs,"./pics/inputs.png");
saveas(figrp,"./pics/randp.png");
% saveas(figpitch,"./pics/pitch.png");