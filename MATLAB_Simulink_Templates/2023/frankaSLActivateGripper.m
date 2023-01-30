function frankaSLActivateGripper(state)

persistent gripAct gripGoal gripperCommand

if isempty(gripAct) || ~isvalid(gripAct)
     pause(1)
    [gripAct,gripGoal] = rosactionclient('/franka_gripper/gripper_action');
    gripperCommand = rosmessage(gripAct);
     pause(3);
end

    
   if state == 1
               % Activate gripper
                %pause(1);
                gripperCommand.Command.Position = 0.04; % 0.04 fully open, 0 fully closed
                gripperCommand.Command.MaxEffort = 50;
                gripGoal.Command = gripperCommand.Command;            
                %pause(1);
                disp(gripAct)
                % Send command
                sendGoal(gripAct,gripGoal); 
                disp('Gripper closed...');
   elseif state == 0 
               % Deactivate gripper
                %pause(1);
                gripperCommand.Command.Position = 0; % 0.04 fully open, 0 fully closed
                gripperCommand.Command.MaxEffort = 50;
                gripGoal.Command = gripperCommand.Command;  
                %pause(1);
                % Send command
                sendGoal(gripAct,gripGoal);
                disp('Gripper open...');
    end
end