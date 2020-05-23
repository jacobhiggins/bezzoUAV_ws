function [ Plant ] = linModel(state,mpcparams,quadparams)
%     addpath('utils');
%     mpcparams = mpc_params();
%     quadparams = nanoplus();
    
    g = quadparams.grav;
    m = quadparams.mass;
    Ixx = quadparams.I(1,1);
    Iyy = quadparams.I(2,2);
    Izz = quadparams.I(3,3);
    kf = quadparams.kf;
%     km = quad_params.km;
    % Arm Parameters
%     l1x = quad_params.arm_mat(1,1);  l1y = quad_params.arm_mat(1,2);
%     l2x = quad_params.arm_mat(2,1);  l2y = quad_params.arm_mat(2,2);
%     l3x = quad_params.arm_mat(3,1);  l3y = quad_params.arm_mat(3,2);
%     l4x = quad_params.arm_mat(4,1);  l4y = quad_params.arm_mat(4,2);
    Ts = mpcparams.Ts;
    
    A = mpcparams.A;
    B = mpcparams.B;
    C = mpcparams.C;
    D = mpcparams.D;
    
    % state      - 12 x 1, state vector = [x, y, z, phi, theta, psi, xdot, ydot, zdot, p, q, r]

    phi = state(4);
    theta = state(5);
    psi = state(6);
    r = state(12);
    
    % Effect of tilting on xdd and ydd
    A(7,4) = g*sin(psi);
    A(7,5) = g*cos(psi);
    A(8,4) = -g*cos(psi);
    A(8,5) = g*sin(psi);
    
    % pqr derivative to euler angle derivative
    eRpqr = [cos(theta), 0, -cos(phi)*sin(theta);...
        0, 1, sin(phi);...
        sin(theta), 0, cos(phi)*cos(theta)];
    
    pqrRe = inv(eRpqr);
    
    A(4:6,10:12) = pqrRe;
    
%     A(4,10) = cos(theta);
%     A(4,12) = -cos(phi)*sin(theta);
%     A(5,12) = sin(phi);
%     A(6,10) = sin(theta);
%     A(6,12) = cos(phi)*cos(theta);
    
    % Pseudo-torques for pqr being in a rotating frame
    A(10,11) = -r*(Izz - Izz)/Ixx;
    A(11,10) = -r*(Ixx - Izz)/Iyy;
    
    % Rotating body to produce acceleration in x,y
    B(7,:) = kf*(cos(psi)*theta + sin(psi)*phi)/m;
    B(8,:) = kf*(sin(psi)*theta - cos(psi)*phi)/m;
    
    Plant = c2d(ss(A,B,C,D),Ts);
    
end