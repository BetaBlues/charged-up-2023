package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shoulderSubsystem;

public class shoulderStartPos extends CommandBase {
    private shoulderSubsystem shoulderSub;

    public shoulderStartPos(shoulderSubsystem shoulderSub){
        this.shoulderSub = shoulderSub;
    }

    @Override
    public void initialize() {

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
     shoulderSub.move(0.5);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      shoulderSub.stopShoulder();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}
