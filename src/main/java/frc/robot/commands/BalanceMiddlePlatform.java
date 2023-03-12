package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

/**
 *
 */
public class BalanceMiddlePlatform extends CommandBase {
    private static DriveSystem m_drivesystem;
    private static double levelYaw = 3;
    public BalanceMiddlePlatform(DriveSystem sub1)
    {
        m_drivesystem = sub1;
        addRequirements(m_drivesystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            if (m_drivesystem.getYaw()<levelYaw &&  m_drivesystem.getYaw()>-levelYaw) {
                m_drivesystem.stop();
            }
            else if (m_drivesystem.getYaw()<-levelYaw) {
                m_drivesystem.forward(0.6);
            }
            
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return (m_drivesystem.getYaw()>-levelYaw && m_drivesystem.getYaw()<levelYaw);
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

