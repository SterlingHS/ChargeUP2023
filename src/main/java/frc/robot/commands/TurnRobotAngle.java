package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

/**
 *
 */
public class TurnRobotAngle extends CommandBase {
    private static DriveSystem m_drivesystem;
    private double startAngle;
    private double angleToTurn;
    private ProfiledPIDController pidcontroller = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(1,0.6));

    public TurnRobotAngle( DriveSystem sub1, double turnAngle)
    {
        
        m_drivesystem = sub1;
        startAngle = m_drivesystem.getAngle();
        angleToTurn = turnAngle;
        pidcontroller.setGoal(angleToTurn);
        pidcontroller.setTolerance(5);

    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            
            double Output = pidcontroller.calculate(m_drivesystem.getAngle()-startAngle);
            System.out.println("Output: " + Output + " Angle: " + (m_drivesystem.getAngle()-startAngle));
            if (Math.abs(Output)<0.40) {
                if (Output < 0) {
                    Output = -0.4;
                }
                else if (Output>0) {
                    Output = 0.4;
                }
            }
            else if (Math.abs(Output)>0.65) {
                if (Output < 0) {
                    Output = -0.65;
                }
                else if (Output>0) {
                    Output = 0.65;
                }
            }
            System.out.println("Output:" + Output);
            m_drivesystem.arcademDrive(0, -Output);
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            m_drivesystem.stop();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return pidcontroller.atGoal();
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

