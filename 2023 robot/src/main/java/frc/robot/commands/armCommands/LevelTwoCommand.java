package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.extenderSubsystem;
import frc.robot.subsystems.shoulderSubsystem;


public class LevelTwoCommand extends CommandBase {
    private final extenderSubsystem extenderSub;
    private final shoulderSubsystem shoulderSub;
    //private final XboxController manipulator;

    public LevelTwoCommand(extenderSubsystem extenderSub, shoulderSubsystem shoulderSub) {
        this.extenderSub = extenderSub;
        this.shoulderSub = shoulderSub;
        //this.manipulator = manipulator;
      }

      @Override
      public void initialize() {

      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        boolean shoulder = shoulderSub.levelTwo(shoulderSub.getPosition());
        boolean extension = extenderSub.levelTwo(extenderSub.getPosition());
        
        if(shoulder && extension){
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