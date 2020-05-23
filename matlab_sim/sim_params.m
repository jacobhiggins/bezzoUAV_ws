% Set sim params in one function
% Optional arguments:
% 'X_0' = 12x1 array, initial conditions
% 'X0' = initial x
% 'Y0' = initial y
% 'Z0' = initial z
% 'XDot0' = initial xdot
% 'YDot0' = initial ydot
% 'ZDot0' = initial zdot
% 'ObsPos' = 3x1 array, position of obstacle
% 'ObsXSpan' = length of obstacle in the x direction
% 'ObsYSpan' = length of obstacle in the y direction
% 'ObsZSpan' = length of obstacle in the z direction
% 'AvoidInX' = MPC should move quad in the x direction to avoid obs
% 'AvoidInY' = move in y direction
% 'AvoidInZ' = move in z direction
% 'R' = measurement noise
% 'Q' = process noise
% 'Ts' = sampling time
% 'Offset' = input offset, in units of sampling time
% 'PH' = prediction horizon
% 'CH' = control horizon
% 'AH' = action horizon
% 'Xref'
% 'XWeight'
% 'XDotWeight'
% 'XMin'
% 'XMax'
% 'XMinECR'
% 'XMaxECR'
% 'XDotMin'
% 'XDotMax'
% 'XDotMinECR'
% 'XDotMaxECR'
% 'Yref'
% 'YWeight'
% 'YDotWeight'
% 'YMin'
% 'YMax'
% 'YMinECR'
% 'YMaxECR'
% 'YDotMin'
% 'YDotMax'
% 'YDotMinECR'
% 'YDotMaxECR'
% 'Zref'
% 'ZWeight'
% 'ZDotWeight'
% 'ZMin'
% 'ZMax'
% 'ZMinECR'
% 'ZMaxECR'
% 'ZDotMin'
% 'ZDotMax'
% 'ZDotMinECR'
% 'ZDotMaxECR'
% 'RollMin'
% 'RollMax'
% 'RollMinECR'
% 'RollMaxECR'
% 'PMin'
% 'PMax'
% 'PMinECR'
% 'PMaxECR'
% 'PitchMin'
% 'PitchMax'
% 'PitchMinECR'
% 'PitchMaxECR'
% 'QMin'
% 'QMax'
% 'QMinECR'
% 'QMaxECR'
% 'YawMin'
% 'YawMax'
% 'YawMinECR'
% 'YawMaxECR'
% 'RMin'
% 'RMax'
% 'RMinECR'
% 'RMaxECR'


function simparams = sim_params(inputs)
    
    try
        args = inputs(1,1:2:end);
%         args = cellfun(@cell2mat,args,'UniformOutput',false);
        args = [inputs{1:2:end}];
        values = {inputs{2:2:end}};
%         values = cellfun(@str2num,values);
    catch ME
        disp("Using default values for sim params");
        args = [];
        values = [];
    end
    
    indexX_0 = find(strcmp([args(:)],'X_0')); %#ok<*NBRAK> % This is the state vector x
    indexX0 = find(strcmp([args(:)],'X0')); % This is the initial x position
    indexY0 = find(strcmp([args(:)],'Y0'));
    indexZ0 = find(strcmp([args(:)],'Z0'));
    indexXDot0 = find(strcmp([args(:)],'XDot0'));
    indexYDot0 = find(strcmp([args(:)],'YDot0'));
    indexZDot0 = find(strcmp([args(:)],'ZDot0'));
    indexObsPos = find(strcmp([args(:)],'ObsPos'));
    indexObsXSpan = find(strcmp([args(:)],'ObsXSpan'));
    indexObsYSpan = find(strcmp([args(:)],'ObsYSpan'));
    indexObsZSpan = find(strcmp([args(:)],'ObsZSpan'));
    indexAvoidInX = find(strcmp([args(:)],'AvoidInX'));
    indexAvoidInY = find(strcmp([args(:)],'AvoidInY'));
    indexAvoidInZ = find(strcmp([args(:)],'AvoidInZ'));
    indexR = find(strcmp([args(:)],'R'));
    indexQ = find(strcmp([args(:)],'Q'));
    indexTs = find(strcmp([args(:)],'Ts'));
    indexOffset = find(strcmp([args(:)],'Offset'));
    indexPH = find(strcmp([args(:)],'PH'));
    indexCH = find(strcmp([args(:)],'CH'));
    indexAH = find(strcmp([args(:)],'AH'));
    indexXref = find(strcmp([args(:)],'Xref'));
    indexXWeight = find(strcmp([args(:)],'XWeight'));
    indexXDotWeight = find(strcmp([args(:)],'XDotWeight'));
    indexXMin = find(strcmp([args(:)],'XMin'));
    indexXMax = find(strcmp([args(:)],'XMax'));
    indexXMinECR = find(strcmp([args(:)],'XMinECR'));
    indexXMaxECR = find(strcmp([args(:)],'XMaxECR'));
    indexXDotMin = find(strcmp([args(:)],'XDotMin'));
    indexXDotMax = find(strcmp([args(:)],'XDotMax'));
    indexXDotMinECR = find(strcmp([args(:)],'XDotMinECR'));
    indexXDotMaxECR = find(strcmp([args(:)],'XDotMaxECR'));
    indexYref = find(strcmp([args(:)],'Yref'));
    indexYWeight = find(strcmp([args(:)],'YWeight'));
    indexYDotWeight = find(strcmp([args(:)],'YDotWeight'));
    indexYMin = find(strcmp([args(:)],'YMin'));
    indexYMax = find(strcmp([args(:)],'YMax'));
    indexYMinECR = find(strcmp([args(:)],'YMinECR'));
    indexYMaxECR = find(strcmp([args(:)],'YMaxECR'));
    indexYDotMin = find(strcmp([args(:)],'YDotMin'));
    indexYDotMax = find(strcmp([args(:)],'YDotMax'));
    indexYDotMinECR = find(strcmp([args(:)],'YDotMinECR'));
    indexYDotMaxECR = find(strcmp([args(:)],'YDotMaxECR'));
    indexZref = find(strcmp([args(:)],'Zref'));
    indexZWeight = find(strcmp([args(:)],'ZWeight'));
    indexZDotWeight = find(strcmp([args(:)],'ZDotWeight'));
    indexZMin = find(strcmp([args(:)],'ZMin'));
    indexZMax = find(strcmp([args(:)],'ZMax'));
    indexZMinECR = find(strcmp([args(:)],'ZMinECR'));
    indexZMaxECR = find(strcmp([args(:)],'ZMaxECR'));
    indexZDotMin = find(strcmp([args(:)],'ZDotMin'));
    indexZDotMax = find(strcmp([args(:)],'ZDotMax'));
    indexZDotMinECR = find(strcmp([args(:)],'ZDotMinECR'));
    indexZDotMaxECR = find(strcmp([args(:)],'ZDotMaxECR'));
    indexRollMin = find(strcmp([args(:)],'RollMin'));
    indexRollMax = find(strcmp([args(:)],'RollMax'));
    indexRollMinECR = find(strcmp([args(:)],'RollMinECR'));
    indexRollMaxECR = find(strcmp([args(:)],'RollMaxECR'));
    indexPMin = find(strcmp([args(:)],'PMin'));
    indexPMax = find(strcmp([args(:)],'PMax'));
    indexPMinECR = find(strcmp([args(:)],'PMinECR'));
    indexPMaxECR = find(strcmp([args(:)],'PMaxECR'));
    indexPitchMin = find(strcmp([args(:)],'PitchMin'));
    indexPitchMax = find(strcmp([args(:)],'PitchMax'));
    indexPitchMinECR = find(strcmp([args(:)],'PitchMinECR'));
    indexPitchMaxECR = find(strcmp([args(:)],'PitchMaxECR'));
    indexQMin = find(strcmp([args(:)],'QMin'));
    indexQMax = find(strcmp([args(:)],'QMax'));
    indexQMinECR = find(strcmp([args(:)],'QMinECR'));
    indexQMaxECR = find(strcmp([args(:)],'QMaxECR'));
    indexYawMin = find(strcmp([args(:)],'YawMin'));
    indexYawMax = find(strcmp([args(:)],'YawMax'));
    indexYawMinECR = find(strcmp([args(:)],'YawMinECR'));
    indexYawMaxECR = find(strcmp([args(:)],'YawMinECR'));
    indexRMin = find(strcmp([args(:)],'RMin'));
    indexRMax = find(strcmp([args(:)],'RMax'));
    indexRMinECR = find(strcmp([args(:)],'RMinECR'));
    indexRMaxECR = find(strcmp([args(:)],'RMaxECR'));
    
    % Make sure num of indices = num of arguments given
    
    % Initial conditions
    if ~isempty(indexX_0) simparams.X_0 = values{indexX_0};...
    else simparams.X_0 = zeros(12,1); end %#ok<*SEPEX>
    if ~isempty(indexX0) simparams.X0 = values{indexX0};...
    else simparams.X0 = 0; end
    if ~isempty(indexY0) simparams.Y0 = values{indexY0};...
    else simparams.Y0 = 0; end
    if ~isempty(indexZ0) simparams.Z0 = values{indexZ0};...
    else simparams.Z0 = 0; end
    if ~isempty(indexXDot0) simparams.XDot0 = values{indexXDot0};...
    else simparams.XDot0 = 0; end
    if ~isempty(indexYDot0) simparams.YDot0 = values{indexYDot0};...
    else simparams.YDot0 = 0; end
    if ~isempty(indexZDot0) simparams.ZDot0 = values{indexZDot0};...
    else simparams.ZDot0 = 0; end

    % Obs params
    if ~isempty(indexObsPos) simparams.obsparams.pos = values{indexObsPos};...
    else simparams.obsparams.pos = [1,1,1]; end
    if ~isempty(indexObsXSpan) simparams.obsparams.xspan = values{indexObsXSpan};...
    else simparams.obsparams.xspan = 0.25; end
    if ~isempty(indexObsYSpan) simparams.obsparams.yspan = values{indexObsYSpan};...
    else simparams.obsparams.yspan = 0.25; end
    if ~isempty(indexObsZSpan) simparams.obsparams.zspan = values{indexObsZSpan};...
    else simparams.obsparams.zspan = 0.25; end

    % MPC behavior
    if ~isempty(indexAvoidInX) simparams.AvoidInX = values{indexAvoidInX};...
    else simparams.AvoidInX = false; end
    if ~isempty(indexAvoidInY) simparams.AvoidInY = values{indexAvoidInY};...
    else simparams.AvoidInY = false; end
    if ~isempty(indexAvoidInZ) simparams.AvoidInZ = values{indexAvoidInZ};...
    else simparams.AvoidInZ = false; end
    
    % Sim params
    if ~isempty(indexR) simparams.R = values{indexR};...
    else simparams.R = diag(zeros(1,12)); end
    if ~isempty(indexQ) simparams.Q = values{indexQ};...
    else simparams.Q = diag([0.0000001*ones(1,3),zeros(1,9)]); end
    if ~isempty(indexTs) simparams.mpcparams.Ts = values{indexTs};...
    else simparams.mpcparams.Ts = 0.01; end % Sampling time
    if ~isempty(indexOffset) simparams.offset = values{indexOffset};...
    else simparams.mpcparams.offset = 0; end
    
    % MPC params
    if ~isempty(indexPH) simparams.mpcparams.PH = values{indexPH};
    else simparams.mpcparams.PH = 200; end
    if ~isempty(indexCH) simparams.mpcparams.CH = values{indexCH};
    else simparams.mpcparams.CH = 2; end
    if ~isempty(indexAH) simparams.mpcparams.AH = values{indexAH};...
    else simparams.mpcparams.AH = 1; end
    
    % Set X params
    if ~isempty(indexXref) simparams.mpcparams.Xref = values{indexXref};...
    else simparams.mpcparams.Xref = 0; end
    if ~isempty(indexXWeight) simparams.mpcparams.XWeight = values{indexXWeight};...
    else simparams.mpcparams.XWeight = 1000; end
    if ~isempty(indexXDotWeight) simparams.mpcparams.XDotWeight = values{indexXDotWeight};...
    else simparams.mpcparams.XDotWeight = 0; end
    if ~isempty(indexXMin) simparams.mpcparams.XMin = values{indexXMin};...
    else simparams.mpcparams.XMin = -Inf; end
    if ~isempty(indexXMax) simparams.mpcparams.XMax = values{indexXMax};...
    else simparams.mpcparams.XMax = Inf; end
    if ~isempty(indexXMinECR) simparams.mpcparams.XMinECR = values{indexXMinECR};...
    else simparams.mpcparams.XMinECR = 1; end
    if ~isempty(indexXMaxECR) simparams.mpcparams.XMaxECR = values{indexXMaxECR};...
    else simparams.mpcparams.XMaxECR = 1; end
    if ~isempty(indexXDotMin) simparams.mpcparams.XDotMin = values{indexXDotMin};...
    else simparams.mpcparams.XDotMin = -Inf; end
    if ~isempty(indexXDotMax) simparams.mpcparams.XDotMax = values{indexXDotMax};...
    else simparams.mpcparams.XDotMax = Inf; end
    if ~isempty(indexXDotMinECR) simparams.mpcparams.XDotMinECR = values{indexXDotMinECR};...
    else simparams.mpcparams.XDotMinECR = 1; end
    if ~isempty(indexXDotMaxECR) simparams.mpcparams.XDotMaxECR = values{indexXDotMaxECR};...
    else simparams.mpcparams.XDotMaxECR = 1; end
    
    % Set Y parameters
    if ~isempty(indexYref) simparams.mpcparams.Yref = values{indexYref};...
    else simparams.mpcparams.Yref = 0; end
    if ~isempty(indexYWeight) simparams.mpcparams.YWeight = values{indexYWeight};...
    else simparams.mpcparams.YWeight = 1000; end
    if ~isempty(indexYDotWeight) simparams.mpcparams.YDotWeight = values{indexYDotWeight};...
    else simparams.mpcparams.YDotWeight = 0; end
    if ~isempty(indexYMin) simparams.mpcparams.YMin = values{indexYMin};...
    else simparams.mpcparams.YMin = -Inf; end
    if ~isempty(indexYMax) simparams.mpcparams.YMax = values{indexYMax};...
    else simparams.mpcparams.YMax = Inf; end
    if ~isempty(indexYMinECR) simparams.mpcparams.YMinECR = values{indexYMinECR};...
    else simparams.mpcparams.YMinECR = 1; end
    if ~isempty(indexYMaxECR) simparams.mpcparams.YMaxECR = values{indexYMaxECR};...
    else simparams.mpcparams.YMaxECR = 1; end
    if ~isempty(indexYDotMin) simparams.mpcparams.YDotMin = values{indexYDotMin};...
    else simparams.mpcparams.YDotMin = -Inf; end
    if ~isempty(indexYDotMax) simparams.mpcparams.YDotMax = values{indexYDotMax};...
    else simparams.mpcparams.YDotMax = Inf; end
    if ~isempty(indexYDotMinECR) simparams.mpcparams.YDotMinECR = values{indexYDotMinECR};...
    else simparams.mpcparams.YDotMinECR = 1; end
    if ~isempty(indexYDotMaxECR) simparams.mpcparams.YDotMaxECR = values{indexYDotMaxECR};...
    else simparams.mpcparams.YDotMaxECR = 1; end

    % Set Z parameters
    if ~isempty(indexZref) simparams.mpcparams.Zref = values{indexZref};...
    else simparams.mpcparams.Zref = 0; end
    if ~isempty(indexZWeight) simparams.mpcparams.ZWeight = values{indexZWeight};...
    else simparams.mpcparams.ZWeight = 10000; end
    if ~isempty(indexZDotWeight) simparams.mpcparams.ZDotWeight = values{indexZDotWeight};...
    else simparams.mpcparams.ZDotWeight = 0; end
    if ~isempty(indexZMin) simparams.mpcparams.ZMin = values{indexZMin};...
    else simparams.mpcparams.ZMin = -Inf; end
    if ~isempty(indexZMax) simparams.mpcparams.ZMax = values{indexZMax};...
    else simparams.mpcparams.ZMax = Inf; end
    if ~isempty(indexZMinECR) simparams.mpcparams.ZMinECR = values{indexZMinECR};...
    else simparams.mpcparams.ZMinECR = 1; end
    if ~isempty(indexZMaxECR) simparams.mpcparams.ZMaxECR = values{indexZMaxECR};...
    else simparams.mpcparams.ZMaxECR = 1; end
    if ~isempty(indexZDotMin) simparams.mpcparams.ZDotMin = values{indexZDotMin};...
    else simparams.mpcparams.ZDotMin = -Inf; end
    if ~isempty(indexZDotMax) simparams.mpcparams.ZDotMax = values{indexZDotMax};...
    else simparams.mpcparams.ZDotMax = Inf; end
    if ~isempty(indexZDotMinECR) simparams.mpcparams.ZDotMinECR = values{indexZDotMinECR};...
    else simparams.mpcparams.ZDotMinECR = 1; end
    if ~isempty(indexZDotMaxECR) simparams.mpcparams.ZDotMaxECR = values{indexZDotMaxECR};...
    else simparams.mpcparams.ZDotMaxECR = 1; end
    
    % Set Roll parameters
    if ~isempty(indexRollMin) simparams.mpcparams.RollMin = values{indexRollMin};
    else simparams.mpcparams.RollMin = -0.2; end
    if ~isempty(indexRollMax) simparams.mpcparams.RollMax = values{indexRollMax};
    else simparams.mpcparams.RollMax = 0.2; end
    if ~isempty(indexRollMinECR) simparams.mpcparams.RollMinECR = values{indexRollMinECR};
    else simparams.mpcparams.RollMinECR = 0.5; end
    if ~isempty(indexRollMaxECR) simparams.mpcparams.RollMaxECR = values{indexRollMaxECR};
    else simparams.mpcparams.RollMaxECR = 0.5; end
    if ~isempty(indexPMin) simparams.mpcparams.PMin = values{indexPMin};...
    else simparams.mpcparams.PMin = -Inf; end
    if ~isempty(indexPMax) simparams.mpcparams.PMax = values{indexPMax};...
    else simparams.mpcparams.PMax = Inf; end
    if ~isempty(indexPMinECR) simparams.mpcparams.PMinECR = values{indexPMinECR};...
    else simparams.mpcparams.PMinECR = 1; end
    if ~isempty(indexPMaxECR) simparams.mpcparams.PMaxECR = values{indexPMaxECR};...
    else simparams.mpcparams.PMaxECR = 1; end

    % Set Pitch parameters
    if ~isempty(indexPitchMin) simparams.mpcparams.PitchMin = values{indexPitchMin};
    else simparams.mpcparams.PitchMin = -0.2; end
    if ~isempty(indexPitchMax) simparams.mpcparams.PitchMax = values{indexPitchMax};
    else simparams.mpcparams.PitchMax = 0.2; end
    if ~isempty(indexPitchMinECR) simparams.mpcparams.PitchMinECR = values{indexPitchMinECR};
    else simparams.mpcparams.PitchMinECR = 0.5; end
    if ~isempty(indexPitchMaxECR) simparams.mpcparams.PitchMaxECR = values{indexPitchMaxECR};
    else simparams.mpcparams.PitchMaxECR = 0.5; end
    if ~isempty(indexQMin) simparams.mpcparams.QMin = values{indexQMin};...
    else simparams.mpcparams.QMin = -Inf; end
    if ~isempty(indexQMax) simparams.mpcparams.QMax = values{indexQMax};...
    else simparams.mpcparams.QMax = Inf; end
    if ~isempty(indexQMinECR) simparams.mpcparams.QMinECR = values{indexQMinECR};...
    else simparams.mpcparams.QMinECR = 1; end
    if ~isempty(indexQMaxECR) simparams.mpcparams.QMaxECR = values{indexQMaxECR};...
    else simparams.mpcparams.QMaxECR = 1; end

    % Set Yaw parameters
    if ~isempty(indexYawMin) simparams.mpcparams.YawMin = values{indexYawMin};
    else simparams.mpcparams.YawMin = -0.0001; end
    if ~isempty(indexYawMax) simparams.mpcparams.YawMax = values{indexYawMax};
    else simparams.mpcparams.YawMax = 0.0001; end
    if ~isempty(indexYawMinECR) simparams.mpcparams.YawMinECR = values{indexYawMinECR};
    else simparams.mpcparams.YawMinECR = 0.01; end
    if ~isempty(indexYawMaxECR) simparams.mpcparams.YawMaxECR = values{indexYawMaxECR};
    else simparams.mpcparams.YawMaxECR = 0.01; end
    if ~isempty(indexRMin) simparams.mpcparams.RMin = values{indexRMin};...
    else simparams.mpcparams.RMin = -Inf; end
    if ~isempty(indexRMax) simparams.mpcparams.RMax = values{indexRMax};...
    else simparams.mpcparams.RMax = Inf; end
    if ~isempty(indexRMinECR) simparams.mpcparams.RMinECR = values{indexRMinECR};...
    else simparams.mpcparams.RMinECR = 1; end
    if ~isempty(indexRMaxECR) simparams.mpcparams.RMaxECR = values{indexRMaxECR};...
    else simparams.mpcparams.RMaxECR = 1; end

    % Get parameters togeher
    simparams.mpcparams.Weights.OutputVariables = [simparams.mpcparams.XWeight,...
        simparams.mpcparams.YWeight,...
        simparams.mpcparams.ZWeight,...
        0,...
        0,...
        0,...
        simparams.mpcparams.XDotWeight,...
        simparams.mpcparams.YDotWeight,...
        simparams.mpcparams.ZDotWeight,...
        0,0,0];
    
    simparams.mpcparams.Weights.ManipulatedVariables = zeros(1,4);
    simparams.mpcparams.Weights.ManipulatedVariablesRate = zeros(1,4);
    
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
    
    % Linearized state dynamics
    A = [zeros(6,6) eye(6);...
        zeros(6,12)];
    A(8,4) = -g;
    A(7,5) = g;
    simparams.mpcparams.A = A;
    
    B = [zeros(8,4);...
        kf*ones(1,4)/m;...
        kf*[l1x, l2x, -l3x, -l4x]/Ixx;...
        kf*[-l1y, l2y, l3y, -l4y]/Iyy;...
        km*[1, -1, 1, -1]/Izz];
    simparams.mpcparams.B = B;
    
    C = eye(12);
    simparams.mpcparams.C = C;
    
    D = zeros(12,4);
    simparams.mpcparams.D = D;
    
end