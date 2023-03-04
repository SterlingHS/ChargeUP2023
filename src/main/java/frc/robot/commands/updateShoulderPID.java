package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSystem;

public class updateShoulderPID extends CommandBase {
    private ShoulderSystem m_shouldersystem;

    public updateShoulderPID(ShoulderSystem sub1) {
        m_shouldersystem = sub1;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        //m_shouldersystem.updateShoulderSystem();
    }

    @Override
    public void end(boolean interrupted) {}

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


    
}
