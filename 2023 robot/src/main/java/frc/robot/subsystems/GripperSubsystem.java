//bison code -- change later
package frc.robot.subsystems;


import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.SparkMaxPIDController; 
import com.revrobotics.CANSparkMax.IdleMode; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 



import static frc.robot.Constants.k_gripper; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
    
    private final CANSparkMax m_motor; 
    public boolean isOpen;
    
    public GripperSubsystem() {

        m_motor = new CANSparkMax(k_gripper.NEO_SPARKMAX_ID, MotorType.kBrushless);
        isOpen = false; 

    }
    
  
    public void open() {
        m_motor.set(k_gripper.MOTOR_OPEN_SPEED);
        isOpen = true;
    }

    public void closeCone() {
        m_motor.set(k_gripper.MOTOR_CONE_CLOSE_SPEED);
        isOpen = false; 
    }
    public void closeCube(){
        m_motor.set(k_gripper.MOTOR_CUBE_CLOSE_SPEED);
        isOpen = false;
    }
    
    public void stop(){
        m_motor.stopMotor();  
    }

}
