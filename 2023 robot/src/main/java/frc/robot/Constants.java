// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public final static int leftFrontMotorID = 2; //2
    public final static int rightFrontMotorID = 5; //5
    public final static int rightRearMotorID = 1;
    public final static int leftRearMotorID = 0;//0

    //chassis speeds
    public final static double normalDriveSpeed = 0.25;
    public final static double normalSidewaysDriveSpeed = 0.35;
    public final static double normalRotationSpeed = 0.3;

    public final static double gyro = 0;

    public final static double inPerSecSpeed = 1;
  }

  public static class gripperConstants {
    public static final int gripperNeoID = 9;

    public static final int gripperID = 9;
    public static final double gripperP = 0.0;
    public static final double gripperD = 0.0;
    public static final double gripperI = 0.0;

    public static final int start = 360;
    public static final int open = 180;
    public static final int coneClose = 45;
    public static final int cubeClose = 90;

    public static final double gripperSpeed = 0.05;
    

}

public static final class ShoulderConstants {
  public static final int shoulderSparkMaxID = 9;
  public static final double shoulderArmP = 0.0;
  public static final double shoulderArmD = 0.0;
  public static final double shoulderArmI = 0.0;
  public static final double stopSpeed = 0.0;
  public static final double shoulderFactor = 185.7142857;
}

public static final class ExtenderConstants {
  public static final int extenderSparkMaxID = 3;
  public static final double extenderArmP = 0.0;
  public static final double extenderArmD = 0.0;
  public static final double extenderArmI = 0.0;
  public static final double extenderFactor = 100.0;
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

//Estimate Distance
public static class EstimateDistanceConstants
{
    final public static double limelightMountAngleDegrees = 15.0; // how many degrees back is your limelight rotated from perfectly vertical?
    final public static double limelightLensHeightInches = 7.0; // distance from the center of the Limelight lens to the floor
    final public static double goalHeightInches = 15.125; // bottom of AprilTag height (same for shoulder and drive use)
}

//Aim and Range
public static class AimAndRangeConstants
{
    final public static double KpAim = -0.1; // Proportional control constant 
    final public static double KpDistance = -0.1; // Proportional control constant for distance
    final public static double min_aim_command = 0.03; // minimum amount of power to move robot
}

//Getting in Range
public static class GettingInRangeConstants
{
    final public static double KpDistance = -0.1; // Proportional control constant for distance
}

//Seeking: None (that I can tell)

//Charge Station
public static class ChargeStationConstants
{
    public static double AprilTagSpacing = 66.00; //in inches
    public static double desiredDistanceCS = 111.00; //in inches --> only for charge station
    public static double limelightOffsetFromRobotCenter = 6.5; //in inches
}

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
  
}
