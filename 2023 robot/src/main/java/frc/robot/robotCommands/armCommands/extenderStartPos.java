package frc.robot.robotCommands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.extenderSubsystem;

public class extenderStartPos extends CommandBase {
    private extenderSubsystem extenderSub;

    public extenderStartPos(extenderSubsystem extenderSub){
        this.extenderSub = extenderSub;
    }

    @Override
    public void initialize() {

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
     extenderSub.move(0.5);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      extenderSub.stopExtender();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}
