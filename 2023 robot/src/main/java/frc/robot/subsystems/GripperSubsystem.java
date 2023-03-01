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



import static frc.robot.Constants.k_gripper; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {
    
    public CANSparkMax motor;
    private SparkMaxPIDController motorController;
    private SparkMaxLimitSwitch limitSwitch;
    private RelativeEncoder encoder; 
 
    public boolean isOpen;
    
    public GripperSubsystem() {

        motor = new CANSparkMax(k_gripper.NEO_SPARKMAX_ID, MotorType.kBrushless);
        isOpen = false;
        
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.k_gripper.open);
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        

        motorController = motor.getPIDController(); 
        limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(360.0);

        motorController.setP(k_gripper.gripperP);
        motorController.setI(k_gripper.gripperP);
        motorController.setD(k_gripper.gripperP);

        motor.setIdleMode(IdleMode.kBrake);

        motor.burnFlash();

    }

    public boolean gripperSwitchState(){
        return limitSwitch.isPressed();
    }

    public void setHome(){
        encoder.setPosition(0);
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void insideThrone(){
        //do later
    }

    public void open(){
        motor.set(.1);
    }
    
    public void closeCone(){
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.k_gripper.coneClose);
        motor.set(-.1);
    }
    public void closeCube(){
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.k_gripper.cubeClose);
        motor.set(-.1);
    }

    public void stop(){
        motor.stopMotor();  
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Gripper LimSwitch", gripperSwitchState());
        SmartDashboard.putNumber("Gripper Position", encoder.getPosition());
    }
}
