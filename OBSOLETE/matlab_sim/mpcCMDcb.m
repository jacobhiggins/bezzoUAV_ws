function mpcCMDcb(~,msg)
    global F;
    global M;
    global mass;
    global grav;
    F = msg.Linear.Z + mass*grav;
    M(1) = msg.Angular.X;
    M(2) = msg.Angular.Y;
    M(3) = msg.Angular.Z;
end