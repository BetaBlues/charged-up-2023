package frc.robot.robotCommands; 

import frc.robot.com.pathplanner.commands.PPMecanumControllerCommand;
import frc.robot.com.pathplanner.lib.PathConstraints;
import frc.robot.com.pathplanner.lib.PathPlanner;
import frc.robot.com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.AllianceUtils; 


import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Stream;


//use PathPlanner to move robot to Pose2d
public class FollowPath implements Command{
    protected final Chassis chassis; 
    protected final Limelight limelight; 
    protected final Map<Supplier<Boolean>, Command> events = new HashMap<>(); 
    protected final Timer timer = new Timer(); 

    protected final PathPoint target; 
    public PathPlannerTrajectory trajectory; 
    
    private PPMecanumControllerCommand mecControllerCommand;

    public FollowPath(Chassis chassis, Limelight limelight, PathPoint target){
        this.chassis = chassis; 
        this.limelight = limelight; 
        this.target = target; 

        trajectory = buildTrajectory(DriveConstants.DEFAULT_PATH_CONSTRAINTS, target);
    }

    @Override
    public void initialize(){
        RobotContainer.field.getObject("Alignment Target").setPose(trajectory.getEndState().poseMeters);
        RobotContainer.field.getObject("Alignment Target").setTrajectory(trajectory);


        mecanumControllerCommand = new PPMecanumControllerCommand(
            trajectory, 
            limelight::getEstimatedPose, 
            new SparkMaxPIDController(DriveConstants.P, DriveConstants.I, DriveConstants.D), chassis::drive); 
        //FIX: wtf

        timer.reset(); 
        timer.start(); 
        mecanumControllerCommand.initialize(); 




    }



    protected PathPlannerTrajectory buildTrajectory(PathConstraints pathConstraints, PathPoint target){
        ArrayList<PathPoint> waypoints = new ArrayList<>(); 

        Pose2d initial = limelight.getEstimatedPose(); 
        Translation2d initialV = 
    }


    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    } 


}