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

    public TurnRobotAngleRight( DriveSystem sub1, double turnAngle)
    {
        m_drivesystem = sub1;
        startAngle = m_drivesystem.getAngle();
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            if (m_drivesystem.getAngle()-startAngle<3.57) {
                m_drivesystem.turnRight();
            }
            else {
                m_drivesystem.stop();
            }
            
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return true;
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

