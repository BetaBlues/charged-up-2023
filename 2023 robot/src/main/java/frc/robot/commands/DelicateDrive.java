package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelicateDrive extends CommandBase {
    private Chassis chassis;
    public double driveSpeed;
    public double rotationSpeed;

    public DelicateDrive(Chassis chassis, double driveSpeed, double rotationSpeed){
        this.chassis = chassis;
        this.driveSpeed = driveSpeed;
        this.rotationSpeed = rotationSpeed;
        addRequirements(chassis);
    }

    @Override 
    public void initialize(){ 
        driveSpeed = Constants.k_chassis.normalDriveSpeed;
        rotationSpeed = Constants.k_chassis.normalRotationSpeed;
    }

    @Override
    public void execute(){
        driveSpeed = Constants.k_chassis.slowDriveSpeed;
        rotationSpeed = Constants.k_chassis.slowRotationSpeed;
    }

}
