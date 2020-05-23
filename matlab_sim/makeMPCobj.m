function MPCobj = makeMPCobj(sysparams,simparams)
    mpcparams = simparams.mpcparams;
    
    kf = sysparams.kf;
    km = sysparams.km;
    
    A = mpcparams.A;
    B = mpcparams.B;
    C = mpcparams.C;
    D = mpcparams.D;
    Ts = mpcparams.Ts;
    
    % Discretize system dynamics
    sys = ss(A,B,C,D);
    sys = c2d(sys,Ts);
    
    % Define input/output names
    sys.InputName = ["del-Quad1", "del-Quad2", "del-Quad3", "del-Quad4"];
    sys.OutputName = ["Xw", "Yw", "Zw", "roll", "pitch", "yaw", "Xw_dot", "Yw_dot", "Zw_dot", "p", "q", "r"];
    sys.InputGroup.MV = [1,2,3,4];
    sys.OutputGroup.MO = 1:12;
    
    MPCobj = mpc(sys,Ts);
    
    MPCobj.Model.Nominal.X = zeros(12,1); % Nominal operating point = hovering
    MPCobj.PredictionHorizon = mpcparams.PH;
    MPCobj.ControlHorizon = mpcparams.CH;
    MPCobj.Weights.OutputVariables = mpcparams.Weights.OutputVariables;
    MPCobj.Weights.ManipulatedVariablesRate = mpcparams.Weights.ManipulatedVariablesRate;
    % Set hard constraints for MV's
    MPCobj.MV(1).Min = -sysparams.mass*sysparams.grav/4/kf; % Min for quadrotors is -mg/4 (delta-u's for MPC)
    MPCobj.MV(2).Min = MPCobj.MV(1).Min;
    MPCobj.MV(3).Min = MPCobj.MV(1).Min;
    MPCobj.MV(4).Min = MPCobj.MV(1).Min;
    MPCobj.MV(1).Max = 1.5*sysparams.mass*sysparams.grav/4/kf; % Max for quadrotors = (2.5mg - 1mg)/4
    MPCobj.MV(2).Max = MPCobj.MV(1).Max;
    MPCobj.MV(3).Max = MPCobj.MV(1).Max;
    MPCobj.MV(4).Max = MPCobj.MV(1).Max;
    % Set soft constraints for MO's
    MPCobj.OV(4).Min = mpcparams.RollMin; % roll min, rad
    MPCobj.OV(5).Min = mpcparams.PitchMin; % pitch min, rad
    MPCobj.OV(6).Min = mpcparams.YawMin; % yaw min, rad
%     MPCobj.OV(7).Min = -1; % x-dot, m/s
    MPCobj.OV(4).Max = mpcparams.RollMax;
    MPCobj.OV(5).Max = mpcparams.PitchMax;
    MPCobj.OV(6).Max = mpcparams.YawMax;
%     MPCobj.OV(7).Max = 1;
    MPCobj.OV(4).MinECR = mpcparams.RollMinECR;
    MPCobj.OV(5).MinECR = mpcparams.PitchMinECR;
    MPCobj.OV(6).MinECR = mpcparams.YawMinECR;
    MPCobj.OV(4).MaxECR = mpcparams.RollMaxECR;
    MPCobj.OV(5).MaxECR = mpcparams.PitchMaxECR;
    MPCobj.OV(6).MaxECR = mpcparams.YawMaxECR;
    
    setEstimator(MPCobj,'custom');
end