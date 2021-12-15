function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Zhu, Yihan

% determine if the desired speed saturate the motors
if (abs(fwdVel) + abs(angVel*wheel2Center) <= maxV)
    cmdV = fwdVel;     % not saturated
    cmdW = angVel;
    return
else
    if (angVel ==0)
        Vel = maxV;
        omega = 0;
    else
        turn_ratio = abs(fwdVel/angVel); % = cmdV/cmdW to keep the same trajectory
        % abs(cmdV) + abs(cmdW * wheel2Center) == maxV
        omega = maxV/(turn_ratio+wheel2Center);
        Vel = omega*turn_ratio;
    end
    % keep the sign of desired vel and ang vel
    if (fwdVel < 0)
        cmdV = -Vel;
    else
        cmdV = Vel;
    end
    if (angVel < 0)
        cmdW = -omega;
    else
        cmdW = omega;
    end
end
