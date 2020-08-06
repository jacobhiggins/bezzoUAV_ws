function mpcparams = mpc_params()
    addpath('utils');
    sysparams = nanoplus();
    g = sysparams.grav;
    m = sysparams.mass;
    Ixx = sysparams.I(1,1);
    Iyy = sysparams.I(2,2);
    Izz = sysparams.I(3,3);
    kf = sysparams.kf;
    km = sysparams.km;
    % Arm Parameters
    l1x = sysparams.arm_mat(1,1);  l1y = sysparams.arm_mat(1,2);
    l2x = sysparams.arm_mat(2,1);  l2y = sysparams.arm_mat(2,2);
    l3x = sysparams.arm_mat(3,1);  l3y = sysparams.arm_mat(3,2);
    l4x = sysparams.arm_mat(4,1);  l4y = sysparams.arm_mat(4,2);

    mpcparams.PH = 200;
    mpcparams.CH = 3;
    mpcparams.AH = 1;
    mpcparams.Ts = 0.01;
    
    % Linearized state dynamics
    A = [zeros(6,6) eye(6);...
        zeros(6,12)];
    A(8,4) = -g;
    A(7,5) = g;
    mpcparams.A = A;
    
    B = [zeros(8,4);...
        kf*ones(1,4)/m;...
        kf*[l1x, l2x, -l3x, -l4x]/Ixx;...
        kf*[-l1y, l2y, l3y, -l4y]/Iyy;...
        km*[1, -1, 1, -1]/Izz];
    mpcparams.B = B;
    
    C = eye(12);
    mpcparams.C = C;
    
    D = zeros(12,4);
    mpcparams.D = D;
    
    mpcparams.Nominal.X = zeros(12,1); % Nominal operating point = hovering
    mpcparams.Weights.OutputVariables = [1000,1000,10000,zeros(1,9)];
    mpcparams.Weights.ManipulatedVariablesRate = zeros(1,4);
    % Set hard constraints for MV's
    mpcparams.MV(1).Min = -sysparams.mass*sysparams.grav/4/kf; % Min for quadrotors is -mg/4 (delta-u's for MPC)
%     mpcparams.MV(1).Min = -Inf;
    mpcparams.MV(2).Min = mpcparams.MV(1).Min;
    mpcparams.MV(3).Min = mpcparams.MV(1).Min;
    mpcparams.MV(4).Min = mpcparams.MV(1).Min;
    mpcparams.MV(1).Max = 1.5*sysparams.mass*sysparams.grav/4/kf; % Max for quadrotors = (2.5mg - 1mg)/4
%     mpcparams.MV(1).Max = Inf;
    mpcparams.MV(2).Max = mpcparams.MV(1).Max;
    mpcparams.MV(3).Max = mpcparams.MV(1).Max;
    mpcparams.MV(4).Max = mpcparams.MV(1).Max;
    % Set soft constraints for MO's
    mpcparams.OV(4).Min = -0.2; % roll min, rad
    mpcparams.OV(5).Min = -0.2; % pitch min, rad
    mpcparams.OV(6).Min = -0.0001; % yaw min, rad
%     MPCobj.OV(7).Min = -1; % x-dot, m/s
    mpcparams.OV(4).Max = 0.2;
    mpcparams.OV(5).Max = 0.2;
    mpcparams.OV(6).Max = 0.0001;
%     MPCobj.OV(7).Max = 1;
    mpcparams.OV(4).MinECR = 0.5;
    mpcparams.OV(5).MinECR = 0.5;
    mpcparams.OV(6).MinECR = 0.5;
    mpcparams.OV(4).MaxECR = 0.5;
    mpcparams.OV(5).MaxECR = 0.5;
    mpcparams.OV(6).MaxECR = 0.5;
end