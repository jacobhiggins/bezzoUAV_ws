function params = iris()
g = 9.81;
% I = [8.795203e-3,   6.39935e-4,        2.5927e-5;
%      6.39935e-4,    5.147140e-3,       7.203e-6;
%      2.5927e-5,     7.203e-6,          1.3625726e-2];

% Sim original values
% m = 0.42 + 1.5; %kg
% I = [2.04016e-5,     0,                0;
%      0,              1.56771e-5,       0;
%      0,              0,                3.51779e-5];
m = 1.282;
I = [8.7952e-3,    6.39935e-4,          0;
     6.39935e-4,             5.14714e-3,   0;
     0,             0,          1.362726e-2];
c = 3.977; % Linear Drag Coefficient

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = 0.086;
params.c = c;

params.arm_mat = [0.22225, 0.22225;
                  0.206375, 0.206375;
                  0.13335, 0.13335;
                  0.13335, 0.13335]; 
              
params.kf = 1.5693e-7;
params.km = 2.7848e-9;
% l_1x l_1y
% l_2x l_2y
% l_3x l_3y
% l_4x l_4y

Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);

params.maxangle = 40 * pi / 180; %you can specify the maximum commanded angle here
params.maxF     = 2.5 * m * g;
 params.minF     = 0.05 * m * g;
end