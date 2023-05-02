// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class ControllerConstants {
    public static final int driverPort = 0;
    public static final int manipulatorPort = 1;
  }
  
  public final class chassisConstants {
    //Chassis Motor ports
    public final static int leftFrontMotorID = 5; //2
    public final static int rightFrontMotorID = 2; //5
    public final static int rightRearMotorID = 1;
    public final static int leftRearMotorID = 0;//0

    //chassis speeds
    public final static double normalDriveSpeed = 0.4; // for comp: 0.6 
    public final static double normalSidewaysDriveSpeed = 0.6; // for comp: 0.77
    public final static double normalRotationSpeed = 0.35; // for comp: 0.5

    public final static double gyro = 0;

    public final static double inPerSecSpeed = 1;
  }                                                    

  public static class gripperConstants {

    public static final int gripperID = 9;
    public static final double gripperP = 0.0;
    public static final double gripperD = 0.0;

    
    public static final double gripperI = 0.0;

    public static final int start = 360;
    public static final int open = 0;
    public static final int coneClose = 80;
    public static final int cubeClose = 75;

    public static final double gripperSpeed = -0.2;

    public static final double gripperCloseSpeed = 0.1;
    public static final double gripperOpenSpeed = -0.13;
    
    

}

public static final class ShoulderConstants {
  public static final int shoulderSparkMaxID = 8;
  public static final double shoulderArmP = 0.0;
  public static final double shoulderArmD = 0.0;
  public static final double shoulderArmI = 0.0;
  public static final double stopSpeed = 0.0;
  public static final double shoulderFactor = 185.7142857;
  public static final int limitSwitchChannel = 9; 
}

public static final class ExtenderConstants {
  public static final int extenderSparkMaxID = 45;
  public static final double extenderArmP = 0.0;
  public static final double extenderArmD = 0.0;
  public static final double extenderArmI = 0.0;
  //public static final double extenderFactor = 100.0;
}

public static class LevelConstants {
  //angle constants
  public static final double cone_levelOneRef_shoulder = 0.0;
  public static final double cone_levelOneTarget_shoulder = 49.2678933;

  public static final double cone_levelOneRef_extender = 0.0;
  public static final double cone_levelOneTarget_extender = 0.0;

  public static final double cone_levelTwoRef_shoulder = 0.0;
  public static final double cone_levelTwoTarget_shoulder = 55.76253722;
  public static final double cone_levelTwoRef_extender = 0.0;
  public static final double cone_levelTwoTarget_extender = 0.0;
}

public static class DriveConstants{

  public static final Rotation2d MODULE_ANGLE_LEFT = new Rotation2d(Math.PI / 4); 
  public static final Rotation2d MODULE_ANGLE_RIGHT = new Rotation2d(-Math.PI / 4); 
  public static final Translation2d[] MODULE_POSITIONS = new Translation2d[]{
    new Translation2d(43.16, MODULE_ANGLE_LEFT),
    new Translation2d(43.16, MODULE_ANGLE_LEFT),
    new Translation2d(-43.16, MODULE_ANGLE_RIGHT),
    new Translation2d(-43.16, MODULE_ANGLE_RIGHT)

    //haven't calculated distance measurements yet 
  };

  public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(MODULE_POSITIONS[0], MODULE_POSITIONS[1], MODULE_POSITIONS[2], MODULE_POSITIONS[3]); 
  public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.005); 


  public static final Rotation2d[] MODULE_ROTATIONS = new Rotation2d[]{
    Rotation2d.fromDegrees(-90), 
    Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(180),
    Rotation2d.fromDegrees(90)
  };
}

/*
//Estimate Distance
public static class EstimateDistanceConstants
{
    final public static double limelightMountAngleDegrees = -45.0; // how many degrees back is your limelight rotated from perfectly vertical?
    final public static double limelightLensHeightInches = 27.5; // distance from the center of the Limelight lens to the floor
    final public static double goalHeightInches = 15.125; // bottom of AprilTag height (same for shoulder and drive use)
}

//Getting in Range
public static class GettingInRangeConstants
{
    final public static double KpDistance = -0.1; // Proportional control constant for distance
}

//Charge Station
public static class ChargeStationConstants
{
    public static double AprilTagSpacing = 66.00; //in inches
    public static double desiredDistanceCS = 132.25; //in inches --> only for charge station --> 111 + 21.25
    public static double limelightOffsetFromRobotCenter = 4.625; //in inches
}

public static class ShoulderDriveConstants
{
  public static double nodeSpacingFromAprilTag = 22.00; //inches
  public static double desiredDistanceSD = 35.75; //inches --> SD = Shoulder Drive --> 14.5 + 21.25
  public static double limelightOffsetFromRobotCenter = 4.625; //inches
}
*/

public static class ButtonConstants {
  public static final int levelOneButton = XboxController.Button.kA.value;
  public static final int levelTwoButton = XboxController.Button.kY.value;
}

  public final class xboxConstants {
    //button numbers from 2021-22 Robot Code
        public static final int buttonA = 2;
        public static final int buttonLeftBumper = 5;
        public static final int buttonRightBumper = 6;
        public static final int buttonB = 3;
        public static final int buttonY = 4;
        public static final int buttonLeftLowerBumper = 7;
        public static final int buttonRightLowerBumper = 8;
        public static final int buttonBack = 9;

        public static final int buttonStart = 10;
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 4;
        public static final int rightYAxis = 5;
        public static final int buttonX = 1;
    }

    public final static class VisionConstants{
      public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);
    }
  
}

















