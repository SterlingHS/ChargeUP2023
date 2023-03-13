package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

/**
 *
 */
public class BalanceMiddlePlatform extends CommandBase {
    private DriveSystem m_drivesystem;
    private double levelYaw = 3;
    private boolean inErrorRange;
    private boolean hasYawChanged;
    private double startYaw;
    private double yawTol;
    private double angle;
    private double climbSpeed;
    public BalanceMiddlePlatform(DriveSystem sub1)
    {
        m_drivesystem = sub1;
        yawTol = 2;
        startYaw = m_drivesystem.getYaw();
        inErrorRange = m_drivesystem.getYaw()<levelYaw &&  m_drivesystem.getYaw()>-levelYaw;
        hasYawChanged = m_drivesystem.getYaw()-startYaw>yawTol;
        climbSpeed = 0.4;
        addRequirements(m_drivesystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            angle = m_drivesystem.getYaw()-startYaw;
            if (!hasYawChanged) {
                m_drivesystem.forward(0.6);
                if (angle>32 && angle<36) {
                    hasYawChanged = true;
                }
            }
            else if (angle>17) {
                m_drivesystem.forward(0.6);
            }
            else if (13<angle && angle<17) {
                m_drivesystem.forward(climbSpeed);
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
            return (inErrorRange);
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

