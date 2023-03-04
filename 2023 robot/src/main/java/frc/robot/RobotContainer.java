// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.xboxConstants;
import frc.robot.commands.armCommands.LevelOneCommand;
import frc.robot.commands.armCommands.LevelTwoCommand;
import frc.robot.commands.armCommands.extenderCommand;
import frc.robot.commands.armCommands.shoulderUpCommand;
import frc.robot.commands.gripperCommands.GripperConeCommand;
import frc.robot.commands.gripperCommands.GripperCubeCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.shoulderSubsystem;
import frc.robot.subsystems.extenderSubsystem;

public class RobotContainer {
  public final Chassis chassis = new Chassis();
  public final GripperSubsystem gripper = new GripperSubsystem();

  //creates manipulator controller
  public final static XboxController manipulator = new XboxController(ControllerConstants.manipulatorPort);
  public final static shoulderSubsystem shoulderSubsystem = new shoulderSubsystem();
  public final static extenderSubsystem extenderSubsystem = new extenderSubsystem();

  //creates controller
  public final static XboxController driver = new XboxController(ControllerConstants.driverPort); //connect XboxController to port 0
  public final static ADXRS450_Gyro gyro = new ADXRS450_Gyro(); // Creates an ADXRS450_Gyro object on the onboard SPI port

  public RobotContainer() {

    chassis.setDefaultCommand(new RunCommand(() -> chassis.driveCartesian(
        driver.getRawAxis(xboxConstants.rightXAxis) * Constants.chassisConstants.normalDriveSpeed,
        driver.getRawAxis(xboxConstants.leftXAxis) * Constants.chassisConstants.normalSidewaysDriveSpeed,
        driver.getRawAxis(xboxConstants.leftYAxis) * Constants.chassisConstants.normalRotationSpeed * -1 // the remote says that forwards is -1 and backwards is 1
        //gyro.getRotation2d()
        ),
        chassis)); 

    shoulderSubsystem.setDefaultCommand(new shoulderUpCommand(shoulderSubsystem, manipulator));
    extenderSubsystem.setDefaultCommand(new extenderCommand(extenderSubsystem, manipulator));

    //chassis.setDefaultCommand(new RunCommand(() -> chassis.driveCartesian(0, .5, 0), chassis));
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    
    // new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
    //   .onTrue(new GripperConeCommand(gripper));
    
    // new JoystickButton(driver, XboxController.Button.kRightBumper.value)
    //   .onTrue(new GripperCubeCommand(gripper));

    new JoystickButton(manipulator, ButtonConstants.levelOneButton).onTrue(new LevelOneCommand(extenderSubsystem, shoulderSubsystem));
    new JoystickButton(manipulator, ButtonConstants.levelTwoButton).onTrue(new LevelTwoCommand(extenderSubsystem, shoulderSubsystem));
  }

  public Command getAutonomousCommand() {
    //return m_autoCommand;
    return new RunCommand(() -> chassis.driveCartesian(0, .3, 0), chassis).withTimeout(2) 
        .andThen(new RunCommand(() -> chassis.driveCartesian(0, 0, 0.3), chassis).withTimeout(2))
        .andThen(new RunCommand(() -> chassis.driveCartesian(0,0,0), chassis));
  } 

}