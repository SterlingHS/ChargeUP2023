package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

/**
 *
 */
public class armResetEncoder extends CommandBase {
    private static ArmSystem armSystem;
    public armResetEncoder( ArmSystem armSys)
    {
        armSystem = armSys;
        addRequirements(armSystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            armSystem.resetPosition();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return true;
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    
}
