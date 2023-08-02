function [q,qd,qdd,trajTimes] = MWRoboCupChallenge_computeTrajectory(currentRobotJConfig, taskFinal, robot, endEffector, trajDuration)
% Copyright 2021 MathWorks, Inc
%
% This function use features available in the Robotics System Toolbox to
% compute a smooth trajectory between the desired end effector position and
% the current robot configuration
%
% For more information on the features implemented here visit:
%   https://www.mathworks.com/help/robotics/referencelist.html?type=function&category=manipulators


        timestep = 0.1;
        ik = inverseKinematics('RigidBodyTree',robot);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1 1 1 1 1 1];

        %Initial task config
        jointInit = wrapToPi(currentRobotJConfig');
        % Initial end-effector pose
        taskInit = getTransform(robot, jointInit, endEffector);

        % Time intervals
        timeInterval = [0;trajDuration];
        trajTimes = timeInterval(1):timestep:timeInterval(end);

        % Retrieve task configurations between initial and final
        [s,sd,sdd] = trapveltraj(timeInterval',numel(trajTimes));
        [T, ~, ~] = transformtraj(taskInit,taskFinal,timeInterval,trajTimes, 'TimeScaling',[s;sd;sdd]/timeInterval(end));

        % Compute corresponding joint configurations
        robotPos = zeros(size(T,3),numel(jointInit));
        initialGuess = wrapToPi(jointInit);
        for i=1:size(T,3)            
            robotPos(i,:) = ik(endEffector,T(:,:,i),weights,initialGuess);
            robotPos(i,:) = wrapToPi(robotPos(i,:));
            initialGuess = robotPos(i,:);            
        end   

        %%  Compute joint velocities and accelerations at required rate for execution by the robot
        % disp('Done planning trajectory, now sampling...')

        % Interpolated joint velocities
        h = timestep;
        robotVelTemp = diff(robotPos)/h;
        robotVel= [zeros(1,numel(jointInit));robotVelTemp];

        % Interpolated joint accelerations
        robotAccTemp = diff(robotVelTemp)/h;
        robotAcc = [zeros(2,numel(jointInit));robotAccTemp];

        q = robotPos';
        qd = robotVel';
        qdd = robotAcc';    
        

end