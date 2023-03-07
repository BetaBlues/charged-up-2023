// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Chassis extends SubsystemBase {
  //Creates new Chassis
  public WPI_VictorSPX leftFrontMotor;
  public WPI_VictorSPX leftRearMotor;
  public WPI_VictorSPX rightFrontMotor;
  public WPI_VictorSPX rightRearMotor;
  private MecanumDrive driveTrain;
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


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

  // ------------------------------------------------------
  // LIMELIGHT STUFF !!!

  public void isTargetFound() 
  { 
      double tv = table.getEntry("tv").getDouble(0);
  }
  
  public long AprilTagFoundID() // returns ApirlTag ID after determining target has been found
  {
      isTargetFound();
      
      long AprilTagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0); 
      
      return AprilTagID;
  }
  
  //input ApirlTag ID into desired distance variable

  public void getOnChargeStation() //automatically drive to desired distance from target
  {
      long AprilTagID = AprilTagFoundID();

      double distanceError = 0;
      double drivingAdjust = 0;
      double desiredDistance = Constants.ChargeStationConstants.desiredDistanceCS;
  
      if(AprilTagID == 6 || AprilTagID == 1)
      {
          double tx = table.getEntry("tx").getDouble(0);
          
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          if(tx > 0)
          {
            new RunCommand(() -> driveCartesian(0, 0, 0.3), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.ChargeStationConstants.limelightOffsetFromRobotCenter));
          }
          else if(tx < 0)
          {
            new RunCommand(() -> driveCartesian(0, 0, -0.3), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.ChargeStationConstants.limelightOffsetFromRobotCenter));
          }
          
          //left AprilTagSpacing (negative value)
          new RunCommand(() -> driveCartesian(0, 0, -0.3), this).withTimeout(DistanceToTimeCalculation(Constants.ChargeStationConstants.AprilTagSpacing));
          
          double currentDistance = Estimate_Distance();
          
          distanceError = desiredDistance - currentDistance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError);
          new RunCommand(() -> driveCartesian(0, -0.3, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
          
      }
      else if(AprilTagID == 7 || AprilTagID == 2)
      {
          double tx = table.getEntry("tx").getDouble(0);
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          if(tx > 0)
          {
            new RunCommand(() -> driveCartesian(0, 0, 0.3), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.ChargeStationConstants.limelightOffsetFromRobotCenter));
          }
          else if(tx < 0)
          {
            new RunCommand(() -> driveCartesian(0, 0, -0.3), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.ChargeStationConstants.limelightOffsetFromRobotCenter));
          }
          
          double current_distance = Estimate_Distance();
          
          distanceError = desiredDistance - current_distance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError);
          new RunCommand(() -> driveCartesian(0, -0.3, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
          
      }
      else if(AprilTagID == 8 || AprilTagID == 3)
      {
          double tx = table.getEntry("tx").getDouble(0);
          
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          if(tx > 0)
          {
            new RunCommand(() -> driveCartesian(0, 0, 0.3), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.ChargeStationConstants.limelightOffsetFromRobotCenter));
          }
          else if(tx < 0)
          {
            new RunCommand(() -> driveCartesian(0, 0, -0.3), this).withTimeout(DistanceToTimeCalculation(-tx + Constants.ChargeStationConstants.limelightOffsetFromRobotCenter));
          }
          
          //right AprilTagSpacing (positive value)
          new RunCommand(() -> driveCartesian(0, 0, 0.3), this).withTimeout(DistanceToTimeCalculation(Constants.ChargeStationConstants.AprilTagSpacing));
          
          double currentDistance = Estimate_Distance();
          
          distanceError = desiredDistance - currentDistance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError); 
          new RunCommand(() -> driveCartesian(0, -0.3, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
      }
  }
  
  public double Estimate_Distance() 
  {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

      double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0); //vertical offset

      double angleToGoalDegrees = Constants.EstimateDistanceConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
      
      //calculate distance
      double distanceFromLimelightToGoalInches = (Constants.EstimateDistanceConstants.goalHeightInches - Constants.EstimateDistanceConstants.limelightLensHeightInches)/Math.tan(angleToGoalRadians); //return
      return distanceFromLimelightToGoalInches;
  }
  
  public static double DistanceToTimeCalculation(double distance)
  {
  double time = distance/Constants.chassisConstants.inPerSecSpeed;
  return time;
  }



}