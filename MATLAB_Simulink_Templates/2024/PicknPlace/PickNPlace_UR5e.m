%% Pick and Place Starter Code
%
% This script shows how to use the motion control utilities to command the
% robot to pickup a bottle and drop it off at bin by using a sequential
% approach

% Execute each section individually and visualize result in Gazebo

%% Setup
rosIP = "192.168.10.102"; % Change IP address!!    % IP address of ROS enabled machine  
rosshutdown; % shut down existing connection to ROS
rosinit(rosIP,11311);
% rosinit(rosIP,'NodeHost','192.168.206.1')

[gripAct,gripGoal] = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory')
gripAct.FeedbackFcn = [];

gripPos = 0.115; %0 is fully closed, 4 is fully open
gripGoal=packGripGoal(gripPos,gripGoal)
sendGoal(gripAct,gripGoal);

[trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory') 
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states")
jointStateMsg = jointSub.LatestMessage

UR5e = loadrobot('universalUR5e')
initialIKGuess = homeConfiguration(UR5e);

% Adjust body transformations from previous URDF version
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));
tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation

%% Set Init Configuration
% A good initial condition will help the inverse kinematics algorithm
% maintain the elbow away from the table
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 

%% Send new configuration (Align with bottle)
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.46 -0.07 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 

%% Approach bottle and close gripper
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.07 0.24]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%% Close gripper
gripGoal=packGripGoal(0.517,gripGoal)
sendGoal(gripAct,gripGoal);

%% Lift bottle 
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.07 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 

%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 

%% Open gripper
gripGoal=packGripGoal(0.1,gripGoal)
sendGoal(gripAct,gripGoal);

%% Helper functions
function initialIKGuess = packIKGuess(initialIKGuess,jointStateMsg)
    initialIKGuess(1).JointPosition = jointStateMsg.Position(4); % update configuration in initial guess
    initialIKGuess(2).JointPosition = jointStateMsg.Position(3);
    initialIKGuess(3).JointPosition = jointStateMsg.Position(1);
    initialIKGuess(4).JointPosition = jointStateMsg.Position(5);
    initialIKGuess(5).JointPosition = jointStateMsg.Position(6);
    initialIKGuess(6).JointPosition = jointStateMsg.Position(7);
end


