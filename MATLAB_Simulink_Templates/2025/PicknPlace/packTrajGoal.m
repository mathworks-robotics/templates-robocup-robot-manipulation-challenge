function trajGoal = packTrajGoal(configSoln,trajGoal)
    
    config = [configSoln(3).JointPosition configSoln(2).JointPosition configSoln(1).JointPosition...
    configSoln(4).JointPosition configSoln(5).JointPosition configSoln(6).JointPosition];
    jointWaypointTimes = 3;
    jointWaypoints = config';
    numJoints = size(jointWaypoints,1);
    
    trajGoal.Trajectory.JointNames = {'elbow_joint','shoulder_lift_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
    
    for idx = 1:numJoints
    
        trajGoal.GoalTolerance(idx) = rosmessage('control_msgs/JointTolerance','DataFormat','struct');
        trajGoal.GoalTolerance(idx).Name = trajGoal.Trajectory.JointNames{idx};
        trajGoal.GoalTolerance(idx).Position = 0;
        trajGoal.GoalTolerance(idx).Velocity = 0;
        trajGoal.GoalTolerance(idx).Acceleration = 0;
    
    end
    
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat','struct');
    trajPts.TimeFromStart = rosduration(jointWaypointTimes,'DataFormat','struct');
    trajPts.Positions = jointWaypoints;
    trajPts.Velocities = zeros(size(jointWaypoints));
    trajPts.Accelerations = zeros(size(jointWaypoints));
    trajPts.Effort = zeros(size(jointWaypoints));
    
    trajGoal.Trajectory.Points = trajPts;
end
