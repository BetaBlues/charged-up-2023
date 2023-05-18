// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotCommands.armCommands;

import frc.robot.subsystems.shoulderSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class shoulderUpCommand extends CommandBase {
  private final shoulderSubsystem shoulderSub;
  private final XboxController manipulator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public shoulderUpCommand(shoulderSubsystem shoulderSub, XboxController manipulator) {
    this.shoulderSub = shoulderSub;
    this.manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulderSub);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    shoulderSub.setHome();
    shoulderSub.move(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  double speed = -manipulator.getLeftY() * .5;
       if (speed < 0) {
           speed *= -speed;
       } else {
           speed *= speed;
       }
      shoulderSub.move(speed);
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