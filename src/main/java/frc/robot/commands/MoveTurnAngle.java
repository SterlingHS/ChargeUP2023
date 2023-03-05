package frc.robot.commands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;


/**
 *
 */
    public class MoveTurnAngle extends CommandBase {

    private final DriveSystem drivesystem;
    private ProfiledPIDController pidController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(.1,.1));

    private static double angle_destination, angle_start;

 
    public MoveTurnAngle(DriveSystem sub, double angle1) {
        drivesystem = sub;
        angle_destination = angle1;

        addRequirements(drivesystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        angle_start = drivesystem.getAngle();
        pidController.setGoal(angle_destination-angle_start);
        pidController.setTolerance(3,1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //System.out.println("angle: " + speed);
        drivesystem.turn(pidController.calculate(drivesystem.getAngle()-angle_start));
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