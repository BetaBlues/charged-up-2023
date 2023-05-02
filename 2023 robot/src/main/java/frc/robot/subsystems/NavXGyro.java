package frc.robot.subsystems;
 

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class NavXGyro implements Gyro{
    private final AHRS navX = new AHRS(); 
   

    public NavXGyro(){
       navX.calibrate(); 
        reset(); 
    }

    public boolean isConnected(){
        return navX.isConnected(); 
    }


    public void calibrate(){
        
    }

    @Override
    public Rotation2d getAngle(){
       return navX.getRotation2d();

    }

    public double getRate(){
         
    }

    public Rotation2d getRotation2d(){

    }

    public void reset(){
        navX.reset(); 
    }

    public void close(){

    }

}
