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
    private ProfiledPIDController pidController = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(0.3,0.3));

    private double distance_destination, distance_start;

 
    public MoveDistance(DriveSystem sub, double distance1) {
        drivesystem = sub;
        distance_destination = distance1;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // System.out.println("something");
        pidController.reset(0, 0);
        distance_start = drivesystem.getDistance();
        System.out.println(distance_destination);
        pidController.setGoal(distance_destination);
        System.out.println(pidController.getGoal());
        pidController.setTolerance(0.2,0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double d = drivesystem.getDistance()-distance_start;
        double distance_from_goal = distance_destination-d;
        System.out.println("distance: " + d + " output: " + pidController.calculate(drivesystem.getDistance()-distance_start));
        drivesystem.forward(pidController.calculate(d));
        /*if (distance_from_goal>=0) {
            drivesystem.forward(pidController.calculate(d));
        }
        else {
            drivesystem.forward(-pidController.calculate(d));
        }*/
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivesystem.stop();
        System.out.println("End of MoveDistance");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pidController.atGoal();
    }
}