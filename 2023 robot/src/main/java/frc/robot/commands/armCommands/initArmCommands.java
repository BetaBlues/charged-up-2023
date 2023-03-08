package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.extenderSubsystem;
import frc.robot.subsystems.shoulderSubsystem;


public class initArmCommands extends CommandBase{
    private extenderSubsystem extenderSub;
    private shoulderSubsystem shoulderSub;
    
    private Command extenderStartPos;
    private Command shoulderStartPos;

    public Command startPos;

    public initArmCommands( extenderSubsystem extenderSub,shoulderSubsystem shoulderSub){
        this.extenderSub = extenderSub;
        this.shoulderSub = shoulderSub;
        extenderStartPos = new extenderStartPos(extenderSub);
        shoulderStartPos = new shoulderStartPos(shoulderSub);
    }

    public Command startingPos() {
       startPos = (Command) shoulderStartPos.withTimeout(5.0).andThen(extenderStartPos.withTimeout(4.0));
        return startPos;
        
    }

    // @Override
    //   public void initialize() {

    //   }
    
    //   // Called every time the scheduler runs while the command is scheduled.
    //   @Override
    //   public void execute() {
        
    //   }
    
    //   // Called once the command ends or is interrupted.
    //   @Override
    //   public void end(boolean interrupted) {
    //     shoulderSub.stopShoulder();
    //     shoulderSub.setHome();
    //   }
    
    //   // Returns true when the command should end.
    //   @Override
    //   public boolean isFinished() {
    //     return true;
    //   }

}

