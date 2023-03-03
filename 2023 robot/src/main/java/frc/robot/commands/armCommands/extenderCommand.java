// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import frc.robot.subsystems.extenderSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class extenderCommand extends CommandBase {
  private final extenderSubsystem extenderSub;
  private final XboxController manipulator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public extenderCommand(extenderSubsystem extenderSub, XboxController manipulator) {
    this.extenderSub = extenderSub;
    this.manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extenderSub);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    extenderSub.setHome();
    extenderSub.move(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  double speed = -manipulator.getRightY() * .5;
       if (speed < 0) {
           speed *= -speed;
       } else {
           speed *= speed;
       }
      extenderSub.move(speed);
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