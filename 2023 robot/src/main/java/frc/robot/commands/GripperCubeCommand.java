package frc.robot.commands;
//each command is its own class (open and close commands would be separate)

//initialize (x1), execute (run until isFinished = true), isFinished (returns true --> time to stop; returns false --> continues), end (done with command) --> building blocks for command

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants;


public class GripperCubeCommand extends CommandBase {
    
    //private final XboxController manipulator;
    private final GripperSubsystem gripperSubsystem;

    public GripperCubeCommand(GripperSubsystem gripperSubsystem) {
        //this.manipulator = manipulator;
        this.gripperSubsystem = gripperSubsystem;

        addRequirements(gripperSubsystem);
    }
    
    @Override
    public void initialize(){
        if(gripperSubsystem.isOpen){
            gripperSubsystem.closeCube();
        }
        else{
            gripperSubsystem.open();
        }
    }


    @Override
    public void execute(){
        
           
    }
    
    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.isOpen = !gripperSubsystem.isOpen;
    }

    @Override
    public boolean isFinished() {
        
        
        if(gripperSubsystem.isOpen && gripperSubsystem.getPosition() <= Constants.k_gripper.cubeClose){
            return true;
        }
        else if(!gripperSubsystem.isOpen && gripperSubsystem.getPosition() >= Constants.k_gripper.open){
            return true;
        }
        return false;
    }
}
