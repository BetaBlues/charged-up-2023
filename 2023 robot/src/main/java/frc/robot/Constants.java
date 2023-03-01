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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public final class k_chassis {
    //Chassis Motor ports
    public final static int leftFrontMotorID = 2; //2
    public final static int rightFrontMotorID = 5; //5
    public final static int rightRearMotorID = 1;
    public final static int leftRearMotorID = 0;//0

    //chassis speeds
    
    //public double driveSpeed = 0;
    //public double rotationSpeesd = 0;

    public final static double normalDriveSpeed = 0.25;
    public final static double normalSidewaysDriveSpeed = 0.35;
    public final static double slowDriveSpeed = 0.1;
    public final static double normalRotationSpeed = 0.3;
    public final static double slowRotationSpeed = 0.1;

    public final static double gyro = 0;
  }

  public static class k_gripper {
    public static final int NEO_SPARKMAX_ID = 9; //not final

    public static final double MOTOR_RUN_SPEED = 0.3; //speeds aren't final either
    public static final double MOTOR_CUBE_CLOSE_SPEED = -0.3; 
    public static final double MOTOR_CONE_CLOSE_SPEED = -0.5; 
    
    public static final double OPEN_POSITION= 0.5; //placeholder values
    public static final double CLOSE_POSITION = 1.0;

    public static final double MOTOR_OPEN_SPEED = 0; 

    public static final double coneTime = 1;
    public static final double cubeTime = 1;

    public static final int gripperID = 9;
    public static final double gripperP = 0.0;
    public static final double gripperD = 0.0;
    public static final double gripperI = 0.0;

    public static final int open = 180;
    public static final int coneClose = 45;
    public static final int cubeClose = 90;
    

}

  public final class k_xbox {
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
