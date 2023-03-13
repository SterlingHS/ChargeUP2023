package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import java.lang.Math;


/**
 *
 */
    public class MoveDistanceConstant extends CommandBase {

    private final DriveSystem drivesystem;

    private double distance_destination, distance_start;
    private double driveSpeed;
    private double distanceTol;
    private double distance_from_goal;

 
    public MoveDistanceConstant(DriveSystem sub, double distance1, double speed) {
        driveSpeed = speed;
        drivesystem = sub;
        distance_destination = distance1;
        distanceTol = 0.2;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // System.out.println("something");
        distance_start = drivesystem.getDistance();
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivesystem.forward(driveSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivesystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double distance_moved = Math.abs(drivesystem.getDistance()-distance_start);
        return (distance_moved > distance_destination-distanceTol); 
    }
}