package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;

/**
 *
 */
public class BalanceMiddlePlatform extends CommandBase {
    private DriveSystem m_drivesystem;
    private boolean inErrorRange;
    private boolean hasPitchChanged;
    private int stage = 0;
    private double startPitch;
    private double PitchTol;
    private double angle;
    private double climbSpeed;
    private static final double firstClimbAngle = 20;
    private static final double climbingAngle = 15;
    private static final double lowerClimbingBound = 9;
    public BalanceMiddlePlatform(DriveSystem sub1)
    {
        m_drivesystem = sub1;
        PitchTol = 2;
        startPitch = m_drivesystem.getPitch();
        hasPitchChanged = false; //(m_drivesystem.getPitch()-startPitch) > PitchTol;
        climbSpeed = 0.4;
        addRequirements(m_drivesystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            angle = m_drivesystem.getPitch()-startPitch;
            if (!hasPitchChanged) {
                climbSpeed = 0.6;
                if (angle>firstClimbAngle) {
                    hasPitchChanged = true;
                }
            }
            else if (angle>climbingAngle+PitchTol) {
                climbSpeed = 0.6;
            }
            else if (lowerClimbingBound<angle && angle<climbingAngle+PitchTol) {
                climbSpeed = 0.4;
            }
            else {
                climbSpeed = 0;

            }
            inErrorRange = (angle)<PitchTol;
            /*if (stage == 2) {
                climbSpeed = .4;
            }

            if (stage == 1) {
                if (angle < 20) {//Number TBD
                    stage = 2;
                }
                climbSpeed = .6;
            }

            if (stage == 0) {
                if (angle > 25) {//Number TBD
                    stage = 1;
                }
                climbSpeed = .6;
            }
            */

            m_drivesystem.forward(climbSpeed);
            
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return (hasPitchChanged && inErrorRange);
            // return (stage ==3  && inErrorRange);
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

