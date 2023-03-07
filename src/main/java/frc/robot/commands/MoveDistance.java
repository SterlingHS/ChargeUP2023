package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;


/**
 *
 */
    public class MoveDistance extends CommandBase {

    private final DriveSystem drivesystem;
    private ProfiledPIDController pidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(0.5,0.1));

    private static double distance_destination, distance_start;

 
    public MoveDistance(DriveSystem sub, double distance1) {
        drivesystem = sub;
        distance_destination = distance1;

        addRequirements(drivesystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("something");
        distance_start = drivesystem.getDistance();
        pidController.setGoal(distance_destination);
        pidController.setTolerance(0.1,0.3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double d = drivesystem.getDistance()-distance_start;
        System.out.println("distance: " + d + " output: " + pidController.calculate(drivesystem.getDistance()-distance_start));
        drivesystem.forward(pidController.calculate(drivesystem.getDistance()-distance_start));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivesystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pidController.atGoal();
    }
}