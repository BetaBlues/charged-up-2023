// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.ShoulderConstants;

public class extenderSubsystem extends SubsystemBase {
  /** making variables */
    public CANSparkMax extenderCIM;
    private SparkMaxPIDController extenderController;

    /**
     * constructor
     */
    public extenderSubsystem() {
        /* giving substance to names */
        extenderCIM = new CANSparkMax(ExtenderConstants.extenderSparkMaxID, MotorType.kBrushed);

        extenderController = extenderCIM.getPIDController(); 

        extenderController.setP(ExtenderConstants.extenderArmP);
        extenderController.setI(ExtenderConstants.extenderArmI);
        extenderController.setD(ExtenderConstants.extenderArmD);

    extenderCIM.setIdleMode(IdleMode.kBrake);

    extenderCIM.burnFlash();
  }

 
  public void stopExtender(){
    extenderCIM.set(ShoulderConstants.stopSpeed);
  }
  public void move(double speed) {

    
    extenderCIM.set(speed);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}