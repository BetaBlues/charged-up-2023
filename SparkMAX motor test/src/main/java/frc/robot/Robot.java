/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  //private DifferentialDrive m_myRobot;
  private XboxController controller = new XboxController(0);
  private static final int leftDeviceID = 1; 
  //private static final int rightDeviceID = 3;
  private CANSparkMax m_leftMotor;
  //private CANSparkMax m_rightMotor;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushed);
    //m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushed);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftMotor.restoreFactoryDefaults();
    //m_rightMotor.restoreFactoryDefaults();

   // m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
//
  }

  @Override
  public void teleopPeriodic() {
    //m_myRobot.tankDrive(controller.getLeftY(), controller.getRightY());
    m_leftMotor.set(controller.getLeftY());
  }
}