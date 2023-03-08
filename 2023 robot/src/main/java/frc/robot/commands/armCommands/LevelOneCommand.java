package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shoulderSubsystem;


public class LevelOneCommand extends CommandBase {
    private final shoulderSubsystem shoulderSub;
    //private final XboxController manipulator;

    public LevelOneCommand(shoulderSubsystem shoulderSub) {
        this.shoulderSub = shoulderSub;
        //this.manipulator = manipulator;
      }

      @Override
      public void initialize() {

      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        boolean shoulder = shoulderSub.levelOne(shoulderSub.getPosition());
        if(shoulder){
          isFinished();
        }
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        shoulderSub.stopShoulder();
        shoulderSub.setHome();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return true;
      }

}