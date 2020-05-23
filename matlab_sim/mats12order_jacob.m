% Script to find the discretized linearized dynamics of the 12th order
% quadrotor
% state: x, y, z, phi (roll), theta (pitch), psi (yaw), xdot, ydot, zdot, p, q, r
% input: thrust, roll moment, pitch moment, yaw moment
%% For Iris Model in ROS Simulation
g = 9.81;
Ixx = 8.7952e-3;
Iyy = 5.14714e-3;
Izz = 1.3624726e-2;
m = 1.282; 
psi_0 = 0; % Nominal yaw angle
Ts = 0.1;
A = [zeros(6,6), eye(6); ...
    0 0 0 g*sin(psi_0) g*cos(psi_0) zeros(1,7);...
    0 0 0 -g*cos(psi_0) g*sin(psi_0) zeros(1,7);...
    zeros(4,12)];
B = [zeros(8,4);...
    1/m 0 0 0;...
    0 1/Ixx 0 0;...
    0 0 1/Iyy 0;...
    0 0 0 1/Izz];
C = zeros(1,12);
D = zeros(1,4);
c_sys = ss(A,B,C,D);
d_sys = c2d(c_sys,Ts);
A = d_sys.A;
B = d_sys.B;
disp(d_sys.A);
disp(d_sys.B);


%% For Matlab Simulation
g = 9.81;
Ixx = 2.04016e-5;
Iyy = 1.56771e-5;
Izz = 3.51779e-5;
m = 0.42+1.5; % Taken from nano plus model kg
psi_0 = 0; % Nominal yaw angle
Ts = 0.1;
A = [zeros(6,6), eye(6); ...
    0 0 0 g*sin(psi_0) g*cos(psi_0) zeros(1,7);...
    0 0 0 -g*cos(psi_0) g*sin(psi_0) zeros(1,7);...
    zeros(4,12)];
B = [zeros(8,4);...
    1/m 0 0 0;...
    0 1/Ixx 0 0;...
    0 0 1/Iyy 0;...
    0 0 0 1/Izz];
C = zeros(1,12);
D = zeros(1,4);
c_sys = ss(A,B,C,D);
d_sys = c2d(c_sys,Ts);
disp(d_sys.A);
disp(d_sys.B);

%% Test Enrico MPC output

A = d_sys.A;
B = d_sys.B;

close all;
executable = './test_sys';
json_model = 'json/uav12.json';
vars_name  = 'var';

launched_str = sprintf('%s %s %s',executable,json_model,vars_name);
system(launched_str);
load(strcat(vars_name,'_X'), '-ascii');
load(strcat(vars_name,'_U'), '-ascii');
load(strcat(vars_name,'_steps'), '-ascii');
load(strcat(vars_name,'_time'), '-ascii');

x = var_X(:,1);
xs = x;
for i = 1:length(var_U(1,:))
    u = var_U(:,i);
    x = A*x + B*u;
    xs = [xs x];
end

per_diff = (var_X - xs)./abs(xs)/2;
abs_diff = (var_X - xs);
per_diff(isnan(per_diff))=0;
fig_pd = figure(1);
hold on;
cmap = colormap(cool(6));
for i = 1:12
    if i < 7
       style = '-'; 
    else
       style = '--';
    end
   plot(0.1*(1:length(per_diff)),per_diff(i,:),style,'Color',cmap(mod(i-1,6)+1,:),'LineWidth',2);
end
% plot(0.1*(1:length(per_diff)),per_diff);
legend("X","Y","Z","Roll","Pitch","Yaw","Xdot","Ydot","Zdot","p","q","r","Location","EastOutside");
title("Percent Difference");
ylabel("Percentage Different");
xlabel("Time (s)");
fig_diff = figure(2);
hold on;
cmap = colormap(cool(6));
for i = 1:12
    if i < 7
       style = '-'; 
    else
       style = '--';
    end
   plot(0.1*(1:length(per_diff)),abs_diff(i,:),style,'Color',cmap(mod(i-1,6)+1,:),'LineWidth',2);
end
% plot(0.1*(1:length(per_diff)),per_diff);
legend("X","Y","Z","Roll","Pitch","Yaw","Xdot","Ydot","Zdot","p","q","r","Location","EastOutside");
ylabel("Difference");
xlabel("Time (s)");
title("Difference");

saveas(fig_pd,"./pics/perdiff.png");
saveas(fig_diff,"./pics/diff.png");
