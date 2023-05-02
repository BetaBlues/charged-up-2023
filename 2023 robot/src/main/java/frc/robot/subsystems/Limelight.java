package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*; 

import edu.wpi.first.apriltag.AprilTagFieldLayout; 
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Arrays;


public class Limelight extends Vision{




    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    private final DoubleArraySubscriber botPose; 
    private final DoubleSubscriber cl; 
    private final DoubleSubscriber tl; 

    private final MecanumDrivePoseEstimator poseEstimator;
    private MecanumDriveWheelPositions[] wheelPositions;  

    public Limelight(){
        botPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(null);
        cl = table.getDoubleTopic("cl").subscribe(0);
        tl = table.getDoubleTopic("tl").subscribe(0);
        poseEstimator = new MecanumDrivePoseEstimator(Constants.DriveConstants.DRIVE_KINEMATICS, null, null, null);
    
    }


    public Optional<Vision.Measurement> getMeasurement(){
        TimestampedDoubleArray[] updates = botPose.readQueue(); 

        if(updates.length == 0){
            return Optional.empty(); 
        }

        TimestampedDoubleArray update = updates[updates.length - 1];

        if(Arrays.equals(update.value, new double[6])){
            return Optional.empty();
        }

        double x = update.value[0];
        double y = update.value[1];
        double z = update.value[2];
        
        double roll = Units.degreesToRadians(update.value[3]);
        double pitch = Units.degreesToRadians(update.value[4]);
        double yaw = Units.degreesToRadians(update.value[5]);

        double latency = cl.get() + tl.get(); 

        double timestamp = (update.timestamp * 1e-6) - (latency * 1e-3);
        Pose3d pose = new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));

        return Optional.of(new Measurement(
            timestamp,
            pose,
            Constants.VisionConstants.LIMELIGHT_STD_DEV
        ));
    }

    public void periodic(){
   

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    public Double getTargetX() {
        return tx.getDouble(0.0);
    }

    public Double getTargetY() {
        return ty.getDouble(0.0);
    }

    public Double getTargetArea() {
        return ta.getDouble(0.0);
    }

}