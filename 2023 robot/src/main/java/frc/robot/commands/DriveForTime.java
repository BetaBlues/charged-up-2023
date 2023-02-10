package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveForTime extends CommandBase{
    
    public Timer timer;
    public double seconds;

    public void DriveForTime(){
        //this.seconds = seconds;
        new StartEndCommand(
        () -> Robot.m_robotContainer.m_chassis.driveCartesian(0, Constants.k_chassis.normalDriveSpeed, Constants.k_chassis.normalDriveSpeed),
        () -> Robot.m_robotContainer.m_chassis.driveCartesian(0, 0, 0),
        Robot.m_robotContainer.m_chassis    
        ).withTimeout(seconds);
    }
    
}
