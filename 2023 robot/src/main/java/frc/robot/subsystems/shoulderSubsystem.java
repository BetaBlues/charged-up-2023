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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LevelConstants;
import frc.robot.Constants.ShoulderConstants;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController; 
import edu.wpi.first.wpilibj2.command.PIDSubsystem;


public class shoulderSubsystem extends SubsystemBase {
  /** making variables */
    public CANSparkMax shoulderNEO;
    private SparkMaxPIDController shoulderController;
    private SparkMaxLimitSwitch shoulderSwitch;
    private RelativeEncoder shoulderEncoder; 
    DigitalInput topLimitSwitch = new DigitalInput(9);
    /**
     * constructor
     */
    public shoulderSubsystem() {
        /* giving substance to names */
        shoulderNEO = new CANSparkMax(ShoulderConstants.shoulderSparkMaxID, MotorType.kBrushless);
        
        shoulderController = shoulderNEO.getPIDController(); 
        shoulderSwitch = shoulderNEO.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        shoulderEncoder = shoulderNEO.getEncoder();
        shoulderEncoder.setPositionConversionFactor(100.0);

        
        //shoulderController.setP(ShoulderConstants.shoulderArmP);
        //shoulderController.setI(ShoulderConstants.shoulderArmI);
        //shoulderController.setD(ShoulderConstants.shoulderArmD);

        PIDController shoulderPid = new PIDController(0.0, 0.0, 0.0);

        




    shoulderNEO.setIdleMode(IdleMode.kBrake);

    shoulderNEO.burnFlash();
  }

  /*public void c_levelOne(){
    shoulderController.setReference(LevelConstants.cone_levelOneRef_shoulder, CANSparkMax.ControlType.kPosition);
    shoulderEncoder.setPosition(LevelConstants.cone_levelOneTarget_shoulder);
  }
  */

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
  public boolean shoulderSwitchState() {
    return shoulderSwitch.isPressed();
  }

  //getter
  public double getPosition(){
    return shoulderEncoder.getPosition();
  }

/*commands */
  public void stopShoulder(){
    shoulderNEO.set(ShoulderConstants.stopSpeed);
  }

  public void setHome() {
    shoulderEncoder.setPosition(0.0);
  }

  public void move(double speed) {

  

    if(speed < 0){
      if(topLimitSwitch.get()){
        //going up and limit is tripped so stop

        shoulderNEO.set(0);

      } else{
        //going up but not tripped 
        shoulderNEO.set(speed);
      }
    }else{
      //not going up 
      shoulderNEO.set(speed);
    }
    
  }

//levels
  public boolean levelOne(double currentPos) {
    //shoulderController.setReference(currentPos, CANSparkMax.ControlType.kPosition);
    //shoulderEncoder.setPosition(LevelConstants.cone_levelOneTarget_shoulder);
    if(currentPos != LevelConstants.cone_levelOneTarget_shoulder && currentPos < LevelConstants.cone_levelTwoTarget_shoulder){
      shoulderNEO.set(0.3);
    }
    else if (currentPos != LevelConstants.cone_levelOneTarget_shoulder && currentPos > LevelConstants.cone_levelTwoTarget_shoulder){
      shoulderNEO.set(0.3);
    }
    else {
      SmartDashboard.putBoolean("Level One Arm position shoulder", true);
      return true;
    }
    return false;
  }

  public boolean levelTwo(double currentPos) {
    //shoulderController.setReference(currentPos, CANSparkMax.ControlType.kPosition);
    //shoulderEncoder.setPosition(LevelConstants.cone_levelTwoTarget_shoulder);
    if(currentPos != LevelConstants.cone_levelTwoTarget_shoulder && currentPos < LevelConstants.cone_levelTwoTarget_shoulder){
      shoulderNEO.set(0.3);
    }
    else if (currentPos != LevelConstants.cone_levelTwoTarget_shoulder && currentPos > LevelConstants.cone_levelTwoTarget_shoulder){
      shoulderNEO.set(0.3);
    }
    else {
      SmartDashboard.putBoolean("Level Two Arm position shoulder", true);
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Shoulder LimSwitch", shoulderSwitchState());
    //SmartDashboard.putNumber("Shoulder Position", shoulderEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}