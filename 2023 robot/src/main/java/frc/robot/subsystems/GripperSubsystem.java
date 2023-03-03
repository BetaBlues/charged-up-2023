//bison code -- change later
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController; 
import com.revrobotics.CANSparkMax.IdleMode; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 



import static frc.robot.Constants.gripperConstants; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
    
    public CANSparkMax motor;
    private SparkMaxPIDController motorController;
    private SparkMaxLimitSwitch limitSwitch;
    private RelativeEncoder encoder; 
 
    public boolean isOpen;
    
    public GripperSubsystem() {

        motor = new CANSparkMax(gripperConstants.gripperNeoID, MotorType.kBrushless);
        isOpen = true;
        
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.gripperConstants.open);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        

        motorController = motor.getPIDController(); 
        limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(360.0);
        
        /*
        motorController.setP(gripperConstants.gripperP);
        motorController.setI(gripperConstants.gripperP);
        motorController.setD(gripperConstants.gripperP);
         */
        
        motor.setIdleMode(IdleMode.kBrake);
        
        encoder.setPosition(180);
        motor.set(0);
        
        motor.burnFlash();

    }

    public boolean gripperSwitchState(){
        return limitSwitch.isPressed();
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void insideThrone(){
        //do later
    }

    public void open(){
        motor.set(Constants.gripperConstants.gripperSpeed);
    }
    
    public void closeCone(){
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.gripperConstants.coneClose);
        motor.set(-Constants.gripperConstants.gripperSpeed);
    }
    public void closeCube(){
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.gripperConstants.cubeClose);
        motor.set(-Constants.gripperConstants.gripperSpeed);
    }

    public void stop(){
        motor.stopMotor();  
    }
/*
 * @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Gripper LimSwitch", gripperSwitchState());
        SmartDashboard.putNumber("Gripper Position", encoder.getPosition());
    }
 */
    
}
