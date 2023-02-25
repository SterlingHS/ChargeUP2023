package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PIDShoulderSystem;

/**
 *
 */
public class shoulderResetEncoder extends CommandBase {
    private static PIDShoulderSystem shoulderSystem;
    public shoulderResetEncoder( PIDShoulderSystem shoulderSys)
    {
        shoulderSystem = shoulderSys;
        addRequirements(shoulderSystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            shoulderSystem.resetEncoder();
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

