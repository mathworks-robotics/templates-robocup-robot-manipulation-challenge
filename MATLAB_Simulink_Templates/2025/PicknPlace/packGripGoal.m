function gripGoal=packGripGoal(pos,gripGoal)
    jointWaypointTimes = 3;
    jointWaypoints = [pos]';
    numJoints = size(jointWaypoints,1);
    gripGoal.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};
    gripGoal.GoalTolerance = rosmessage('control_msgs/JointTolerance','DataFormat','struct');
    gripGoal.GoalTolerance.Name = gripGoal.Trajectory.JointNames{1};
    gripGoal.GoalTolerance.Position = 0;
    gripGoal.GoalTolerance.Velocity = 0.1;
    gripGoal.GoalTolerance.Acceleration = 0.1;
    
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat','struct');
    trajPts.TimeFromStart = rosduration(jointWaypointTimes,'DataFormat','struct');
    trajPts.Positions = jointWaypoints;
    trajPts.Velocities = zeros(size(jointWaypoints));
    trajPts.Accelerations = zeros(size(jointWaypoints));
    trajPts.Effort = zeros(size(jointWaypoints));
    
    gripGoal.Trajectory.Points = trajPts;
end