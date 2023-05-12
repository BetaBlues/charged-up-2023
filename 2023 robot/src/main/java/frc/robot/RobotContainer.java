// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.xboxConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.shoulderSubsystem;
import frc.robot.subsystems.extenderSubsystem;


import com.pathplanner.lib.server.PathPlannerServer;

public class RobotContainer {
  public final static Chassis chassis = new Chassis();
  public final static GripperSubsystem gripper = new GripperSubsystem();

  //creates manipulator controller
  public final static XboxController manipulator = new XboxController(ControllerConstants.manipulatorPort);
  public final static shoulderSubsystem shoulderSubsystem = new shoulderSubsystem();
  public final static extenderSubsystem extenderSubsystem = new extenderSubsystem();

  //creates controller
  public final static XboxController driver = new XboxController(ControllerConstants.driverPort); //connect XboxController to port 0
  //public final static ADXRS450_Gyro gyro = new ADXRS450_Gyro(); // Creates an ADXRS450_Gyro object on the onboard SPI port

  public RobotContainer() {
    
    chassis.setDefaultCommand(new RunCommand(() -> chassis.driveCartesian(
        driver.getRawAxis(xboxConstants.rightXAxis) * Constants.chassisConstants.normalRotationSpeed,
        driver.getRawAxis(xboxConstants.leftXAxis) * Constants.chassisConstants.normalSidewaysDriveSpeed,
        driver.getRawAxis(xboxConstants.leftYAxis) * Constants.chassisConstants.normalDriveSpeed * -1 // the remote says that forwards is -1 and backwards is 1
        //gyro.getRotation2d()
        ),
        chassis)); 

    shoulderSubsystem.setDefaultCommand(new RunCommand(() -> shoulderSubsystem.move(manipulator.getRawAxis(xboxConstants.rightYAxis) * 0.15), shoulderSubsystem));
    extenderSubsystem.setDefaultCommand(new RunCommand(() -> extenderSubsystem.move(manipulator.getRawAxis(xboxConstants.leftYAxis) * -0.2), extenderSubsystem));

    //GripperSubsystem.setDefaultCommand(new RunCommand(() -> GripperSubsystem.move(manipulator.getRawButton())))
    //shoulderSubsystem.setDefaultCommand(new shoulderUpCommand(shoulderSubsystem, manipulator));
    //extenderSubsystem.setDefaultCommand(new extenderCommand(extenderSubsystem, manipulator));

    configureButtonBindings();

  }

  private void configureButtonBindings() {

    new JoystickButton(manipulator, XboxController.Button.kRightBumper.value)
      .onTrue(new RunCommand(() -> gripper.move(Constants.gripperConstants.gripperOpenSpeed), gripper)).onFalse(new RunCommand(() -> gripper.stop(), gripper));
    new JoystickButton(manipulator, XboxController.Button.kLeftBumper.value)
     .onTrue(new RunCommand(() -> gripper.move(Constants.gripperConstants.gripperCloseSpeed), gripper)).onFalse(new RunCommand(() -> gripper.stop(), gripper));
    

  }

  public double distanceSecondsY(double distance){
    double conversionY = 1.34/36; // seconds over inches 
    return conversionY * distance;
  }
  public double distanceSecondsX(double distance){
    double conversionX = 0; // seconds over inches 
    return conversionX * distance;
  }

  public Command auto(double position, String color){
    double forwards = Constants.chassisConstants.normalDriveSpeed;
    double backwards = -Constants.chassisConstants.normalDriveSpeed;
    double right = Constants.chassisConstants.normalSidewaysDriveSpeed;
    double left = -Constants.chassisConstants.normalSidewaysDriveSpeed;

    if (position == 1){
      return new RunCommand(() -> chassis.driveY(forwards), chassis).withTimeout(distanceSecondsY(90))
        .andThen(() -> chassis.driveY(backwards), chassis).withTimeout(distanceSecondsY(90));
    }
    else if(position == 2){
      if (color.equals("red")){
        return new RunCommand(() -> chassis.driveX(left), chassis).withTimeout(distanceSecondsX(66))
                .andThen(() -> chassis.driveY(forwards), chassis).withTimeout(distanceSecondsY(60))
                .andThen(() -> chassis.driveY(backwards), chassis).withTimeout(distanceSecondsY(60));    
      }
      else {
        return new RunCommand(() -> chassis.driveX(right), chassis).withTimeout(distanceSecondsX(66))
                .andThen(() -> chassis.driveY(forwards), chassis).withTimeout(distanceSecondsY(60))
                .andThen(() -> chassis.driveY(backwards), chassis).withTimeout(distanceSecondsY(60));    
      }
    }
    else{
      return new RunCommand(()-> chassis.driveY(forwards), chassis).withTimeout(distanceSecondsY(144))
        .andThen(() -> chassis.driveY(backwards), chassis).withTimeout(distanceSecondsY(144));
    }
  }

  public Command getAutonomousCommand() {

     
    Command flipAndBack = new RunCommand(() -> shoulderSubsystem.move(-0.4), shoulderSubsystem).withTimeout(.4)
    .andThen(new RunCommand(() -> shoulderSubsystem.stopShoulder()).withTimeout(0.5))
    .andThen(new RunCommand(() -> chassis.driveCartesian(0, 0, -0.4), chassis).withTimeout(10.5));
    //add withTimeout() at the end, using zero runs it at the same time, using 14.6 causes build to fail

    //Command flipAndBack = new RunCommand(() -> chassis.driveCartesian(0, 0, -0.4), chassis).withTimeout(4);
    return flipAndBack;

  } 
}