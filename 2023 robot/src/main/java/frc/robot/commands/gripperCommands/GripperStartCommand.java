package frc.robot.commands.gripperCommands;
//each command is its own class (open and close commands would be separate)

//initialize (x1), execute (run until isFinished = true), isFinished (returns true --> time to stop; returns false --> continues), end (done with command) --> building blocks for command

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants;


public class GripperStartCommand extends CommandBase {
    
    //private final XboxController manipulator;
    private final GripperSubsystem gripper;

    public GripperStartCommand(GripperSubsystem gripperSubsystem) {
        //this.manipulator = manipulator;
        gripper = gripperSubsystem;

        addRequirements(gripperSubsystem);
    }
    
    @Override
    public void initialize(){
        gripper.setPosition(Constants.gripperConstants.start);
        gripper.move(-Constants.gripperConstants.gripperSpeed);
    }


    @Override
    public void execute(){

    }
    
    @Override
    public void end(boolean interrupted) {
        gripper.stop();
        gripper.setPosition(Constants.gripperConstants.open);

    }

    @Override
    public boolean isFinished() {
        if(gripper.getPosition() <= Constants.gripperConstants.open)
            return true;
        return false;
    }
}
