function MWRoboCupChallenge_setInitialConfig
% Copyright 2021 MathWorks, Inc
%
% DO NOT MODIFY !
%
% Donot modify this initialization function if you are participating in the
% MathWorks RoboCup Manipulation Challenge. The initial robot configuration
% is not meant to be modified.

configClient = rossvcclient('/gazebo/set_model_configuration'); % DO NOT USE this ROS client to control the robot, it will not account for environment physics
configReq = rosmessage(configClient);
configReq.ModelName = "my_gen3";
configReq.UrdfParamName = "/my_gen3/robot_description";
configReq.JointNames = [ "joint_1", "joint_2", "joint_3","joint_4", "joint_5", "joint_6", "joint_7"];
configReq.JointPositions = [ -1.9494   -0.0346   -1.1962   -1.0550    0.0367   -2.0500    1.5847]; % DO NOT CHANGE INITIAL CONFIGURATION, it could affect behavior during final scoring
configResp = call(configClient, configReq, 'Timeout', 3);
end

