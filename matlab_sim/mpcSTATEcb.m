function mpcSTATEcb(~,msg)
    global F;
    global M;
    global mpc_state;
    global mass;
    global grav;
    F = msg.Pose.Covariance(1) + mass*grav;
    M(1) = msg.Pose.Covariance(2);
    M(2) = msg.Pose.Covariance(3);
    M(3) = msg.Pose.Covariance(4);
    
    mpc_state(1) = msg.Pose.Pose.Position.X;
    mpc_state(2) = msg.Pose.Pose.Position.Y;
    mpc_state(3) = msg.Pose.Pose.Position.Z;
    mpc_state(4) = msg.Pose.Pose.Orientation.X; % Roll
    mpc_state(5) = msg.Pose.Pose.Orientation.Y; % Pitch
    mpc_state(6) = msg.Pose.Pose.Orientation.Z; % Yaw
    mpc_state(7) = msg.Twist.Twist.Linear.X;
    mpc_state(8) = msg.Twist.Twist.Linear.Y;
    mpc_state(9) = msg.Twist.Twist.Linear.Z;
    mpc_state(10) = msg.Twist.Twist.Angular.X;
    mpc_state(11) = msg.Twist.Twist.Angular.Y;
    mpc_state(12) = msg.Twist.Twist.Angular.Z;
end