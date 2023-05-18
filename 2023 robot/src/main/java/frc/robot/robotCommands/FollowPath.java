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

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    } 


}