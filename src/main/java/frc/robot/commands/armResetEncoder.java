package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
            System.out.println("Arm reset encoder executed");
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            armSystem.resetPosition();
            System.out.println("Has ended");
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    
}
