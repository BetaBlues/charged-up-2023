// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  //Creates new Chassis
  public WPI_VictorSPX leftFrontMotor;
  public WPI_VictorSPX leftRearMotor;
  public WPI_VictorSPX rightFrontMotor;
  public WPI_VictorSPX rightRearMotor;
  private MecanumDrive driveTrain;

  public Chassis() {
    leftFrontMotor = new WPI_VictorSPX(Constants.chassisConstants.leftFrontMotorID);
    leftRearMotor = new WPI_VictorSPX(Constants.chassisConstants.leftRearMotorID);
    rightFrontMotor = new WPI_VictorSPX(Constants.chassisConstants.rightFrontMotorID);
    rightRearMotor = new WPI_VictorSPX(Constants.chassisConstants.rightRearMotorID);
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);
  
    driveTrain = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
  }

  public void driveCartesian(double zRotation, double ySpeed, double xSpeed) { //, Rotation2d gyroAngle
    driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Object driveCartesian(double leftX, double leftY, double rightX, Chassis m_chassis) {
    return null;
  }
}