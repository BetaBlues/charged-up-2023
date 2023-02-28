// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.k_chassis;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.k_xbox;
import frc.robot.commands.GripperConeCommand;
import frc.robot.commands.GripperCubeCommand;
//import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {
  public final Chassis m_chassis = new Chassis();
  public final GripperSubsystem gripper = new GripperSubsystem();
  //private  final Commands m_command = new Commands();
  //private final AutonomousCommand m_autoCommand = new AutonomousCommand();

  //creates controller
  XboxController m_chassisController = new XboxController(0); //connect XboxController to port 1
  ADXRS450_Gyro gyro = new ADXRS450_Gyro(); // Creates an ADXRS450_Gyro object on the onboard SPI port
  
  //double driveSpeed = Constants.k_chassis.normalDriveSpeed;
  //double rotationSpeed = Constants.k_chassis.normalRotationSpeed;

  public RobotContainer() {

    //m_chassisController.a.onTrue(driveSpeed = Constants.k_chassis.slowDriveSpeed);

    //new method of moving chassis. Eliminates need for Chassis subsystem because the chassis is the defult command
  
    m_chassis.setDefaultCommand(new RunCommand(() -> m_chassis.driveCartesian(
        m_chassisController.getRawAxis(k_xbox.rightXAxis) * Constants.k_chassis.normalDriveSpeed,
        m_chassisController.getRawAxis(k_xbox.leftXAxis) * Constants.k_chassis.normalSidewaysDriveSpeed,
        m_chassisController.getRawAxis(k_xbox.leftYAxis) * Constants.k_chassis.normalRotationSpeed * -1 // the remote says that forwards is -1 and backwards is 1
        //gyro.getRotation2d()
        ),
        m_chassis)); //eventually should add the gyro sensor as a 4th parameter. This will make feild orriented drive work.
    
    //m_chassis.setDefaultCommand(new RunCommand(() -> m_chassis.driveCartesian(0, .5, 0), m_chassis));
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    //add button bindings here
    //final JoystickButton buttonLeftLowerBumper = new JoystickButton(m_chassisController, k_xbox.buttonLeftLowerBumper);
    //final JoystickButton buttonRightLowerBumper = new JoystickButton(m_chassisController, k_xbox.buttonRightLowerBumper);
    
    //Trigger delicateDrive = new JoystickButton(m_chassisController, XboxController.Button.kA.value);
    //JoystickButton testButton = new JoystickButton(m_chassisController, XboxController.Button.kA.value);
    
    new JoystickButton(m_chassisController, XboxController.Button.kB.value)
      .onTrue(new GripperConeCommand(gripper).withTimeout(Constants.k_gripper.coneTime));
    
    new JoystickButton(m_chassisController, XboxController.Button.kA.value)
      .onTrue(new GripperCubeCommand(gripper).withTimeout(Constants.k_gripper.cubeTime));
    

    //testButton.whileTrue(Commands.print("hello!")); //test for buttons 
  
  }
// this was used to test if program can read button commands 
  public Command print(){
    return new RunCommand(() -> System.out.println("hello world"));
  }

  public Command getAutonomousCommand() {
    //return m_autoCommand;
    return new RunCommand(() -> m_chassis.driveCartesian(0, .3, 0), m_chassis).withTimeout(2) 
        .andThen(new RunCommand(() -> m_chassis.driveCartesian(0, 0, 0.3), m_chassis).withTimeout(2))
        .andThen(new RunCommand(() -> m_chassis.driveCartesian(0,0,0), m_chassis));
  } 

}