package frc.robot.commands.unused;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveForTime extends CommandBase{
    
    public Timer timer;
    public double seconds;

    public void DriveForTime(double seconds){
        this.seconds = seconds;
        new StartEndCommand(
        () -> Robot.m_robotContainer.chassis.driveCartesian(0, Constants.chassisConstants.normalDriveSpeed, Constants.chassisConstants.normalDriveSpeed),
        () -> Robot.m_robotContainer.chassis.driveCartesian(0, 0, 0),
        Robot.m_robotContainer.chassis    
        ).withTimeout(seconds);
    }
    
}
