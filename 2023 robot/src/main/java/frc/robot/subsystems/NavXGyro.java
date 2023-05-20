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
   //new navX obj

    public NavXGyro(){
       navX.calibrate();
       //calibrates 
        reset(); 
    }

    public boolean isConnected(){
        return navX.isConnected(); 
    }


    public void calibrate(){
        
    }

    @Override
    public double getAngle(){
       return navX.getAngle();

    }

    public double getRate(){
        return navX.getRate(); 
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(navX.getAngle()); 
    }

    public void reset(){
        navX.reset(); 
    }

    public void close(){

    }

}
