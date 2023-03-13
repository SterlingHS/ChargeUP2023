package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ShoulderSystem;

/**
 *
 */
public class TurnRobotAngleRight extends CommandBase {
    private static DriveSystem m_drivesystem;
    private static double startAngle;
    private double angleToTurn;
    private boolean angleIsFinished;

    public TurnRobotAngleRight( DriveSystem sub1, double turnAngle)
    {
        m_drivesystem = sub1;
        startAngle = m_drivesystem.getAngle();
        angleToTurn = turnAngle;
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            angleIsFinished = (m_drivesystem.getAngle()-startAngle)>angleToTurn;
            if ((m_drivesystem.getAngle()-startAngle)>(angleToTurn-30)) {
                m_drivesystem.turnSpeed(0.5);
            }
            else {
                m_drivesystem.turnRight();
            }
            System.out.println(m_drivesystem.getAngle());
            System.out.println("Turning Right");
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            m_drivesystem.stop();
            System.out.println(m_drivesystem.getAngle()-startAngle);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return angleIsFinished;
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

