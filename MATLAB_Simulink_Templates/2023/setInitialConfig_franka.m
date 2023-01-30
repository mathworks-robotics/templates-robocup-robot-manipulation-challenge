function setInitialConfig_franka
% Copyright 2022 MathWorks, Inc
%
% DO NOT MODIFY !
%
% Donot modify this initialization function if you are participating in the
% MathWorks RoboCup Manipulation Challenge. The initial robot configuration
% is not meant to be modified.

jointMess = rosmessage("geometry_msgs/PoseStamped");
jointPub = rospublisher("/cartesian_impedance_example_controller/equilibrium_pose");
pause(1);

gripperX = 0.4;
gripperY = 0;
gripperZ = 0.4;

gripperRotationX = -pi; % radians
gripperRotationY = -0.01; % radians
gripperRotationZ = 0; % radians

jointMess.Pose.Position.X = gripperX;
jointMess.Pose.Position.Y = gripperY;
jointMess.Pose.Position.Z = gripperZ;
quat = angle2quat(gripperRotationX, gripperRotationY, gripperRotationZ, "XYZ");
jointMess.Pose.Orientation.W = quat(1);
jointMess.Pose.Orientation.X = quat(2);
jointMess.Pose.Orientation.Y = quat(3);
jointMess.Pose.Orientation.Z = quat(4);
send(jointPub,jointMess)
pause(2);
end

