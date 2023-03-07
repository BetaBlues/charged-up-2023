// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.LevelConstants;
import frc.robot.Constants.ShoulderConstants;

public class extenderSubsystem extends SubsystemBase {
  /** making variables */
    public CANSparkMax extenderNEO;
    private SparkMaxPIDController extenderController;
    private SparkMaxLimitSwitch extenderSwitch;
    private RelativeEncoder extenderEncoder; 

    /**
     * constructor
     */
    public extenderSubsystem() {
        /* giving substance to names */
        extenderNEO = new CANSparkMax(ExtenderConstants.extenderSparkMaxID, MotorType.kBrushless);
        
        extenderController = extenderNEO.getPIDController(); 
        extenderSwitch = extenderNEO.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        extenderEncoder = extenderNEO.getEncoder();
        extenderEncoder.setPositionConversionFactor(100.0);

        extenderController.setP(ExtenderConstants.extenderArmP);
        extenderController.setI(ExtenderConstants.extenderArmI);
        extenderController.setD(ExtenderConstants.extenderArmD);

    extenderNEO.setIdleMode(IdleMode.kBrake);

    extenderNEO.burnFlash();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

   /*states */
  public boolean extenderSwitchState() {
    return extenderSwitch.isPressed();
  }

  //getters
  public double getPosition() {
    return extenderEncoder.getPosition();
  }

/*commands */
  public void stopExtender(){
    extenderNEO.set(ShoulderConstants.stopSpeed);
  }

  public void setHome() {
    extenderEncoder.setPosition(0.0);
  }

  public void move(double speed) {
    extenderNEO.set(speed);
  }

  //levels
  public boolean levelOne(double currentPos) {
    //extenderController.setReference(currentPos, CANSparkMax.ControlType.kPosition);
    //extenderEncoder.setPosition(LevelConstants.cone_levelOneTarget_extender);
    if(currentPos != LevelConstants.cone_levelOneTarget_extender && currentPos < LevelConstants.cone_levelTwoTarget_shoulder){
      extenderNEO.set(0.3);
    }
    else if (currentPos != LevelConstants.cone_levelOneTarget_extender && currentPos > LevelConstants.cone_levelTwoTarget_shoulder){
      extenderNEO.set(0.3);
    }
    else {
      SmartDashboard.putBoolean("Level One Arm position extender", true);
      return true;
    }
    return false;
  }

  public boolean levelTwo(double currentPos) {
    //extenderController.setReference(currentPos, CANSparkMax.ControlType.kPosition);
    //extenderEncoder.setPosition(LevelConstants.cone_levelOneTarget_extender);
    if(currentPos != LevelConstants.cone_levelTwoTarget_extender && currentPos < LevelConstants.cone_levelTwoTarget_shoulder){
      extenderNEO.set(0.3);
    }
    else if (currentPos != LevelConstants.cone_levelTwoTarget_extender && currentPos > LevelConstants.cone_levelTwoTarget_shoulder){
      extenderNEO.set(0.3);
    }
    else {
      SmartDashboard.putBoolean("Level Two Arm position extender", true);
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Extender LimSwitch", extenderSwitchState());
    SmartDashboard.putNumber("Extender Position", extenderEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}