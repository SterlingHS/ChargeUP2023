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
    private ProfiledPIDController pidController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(.1,.1));

    private static double distance_destination, distance_start;

 
    public MoveDistance(DriveSystem sub, double distance1) {
        drivesystem = sub;
        distance_destination = distance1;

        addRequirements(drivesystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        distance_start = drivesystem.read_distance_encoder();
        pidController.setGoal(distance_destination-distance_start);
        pidController.setTolerance(3,3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //System.out.println("angle: " + speed);
        drivesystem.turn(pidController.calculate(drivesystem.read_distance_encoder()-distance_start));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivesystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}