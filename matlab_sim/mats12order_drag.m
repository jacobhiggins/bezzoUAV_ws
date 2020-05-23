% Script to find the discretized linearized dynamics of the 12th order
% quadrotor
% state: x, y, z, phi (roll), theta (pitch), psi (yaw), xdot, ydot, zdot, p, q, r
% input: thrust, roll moment, pitch moment, yaw moment
g = 9.81;
Ixx = 2.04016e-4;
Iyy = 2.321e-4;
Izz = 3.738e-4;
m = 0.176; % Taken from nano plus model kg
cxy = 3.977; % Linear drag coefficient for xy direction
cz = 3.977; % Linear drag coefficient for z direction
psi_0 = 0; % Nominal yaw angle
Ts = 0.1;
A = [zeros(6,6), eye(6); ...
    0 0 0 g*sin(psi_0) g*cos(psi_0) 0 -cxy zeros(1,5);...
    0 0 0 -g*cos(psi_0) g*sin(psi_0) 0 0 -cxy zeros(1,4);...
    zeros(1,8) -cz zeros(1,3);...
    zeros(3,12)];
B = [zeros(8,4);...
    1/m 0 0 0;...
    0 1/Ixx 0 0;...
    0 0 1/Iyy 0;...
    0 0 0 1/Izz];
C = zeros(1,12);
D = zeros(1,4);
c_sys = ss(A,B,C,D);
d_sys = c2d(c_sys,Ts);
d_sys.A
d_sys.B

%% Test Drag Coefficients
close all;
A = d_sys.A;
B = d_sys.B;
x = [0,0,0,0,0,0,0,0,0,0,0,0]';
T = 5;
xs = x;
for t = 0.1:Ts:T
   if t == 0.1
    x(4) = -0.05;
   else
    x(4) = 0;
   end
   x = A*x + B*u;
   xs = [xs x];
end

plot(0:0.1:T,xs(2,:),'LineWidth',2);
ylabel("X Position (m)");
xlabel("Time (s)");
title("Impulse along Z on System with Drag");