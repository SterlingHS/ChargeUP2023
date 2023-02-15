package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSystem;

public class lowerShoulder extends CommandBase {
    private final ShoulderSystem m_shoulder;

    public lowerShoulder(ShoulderSystem sub1) {
        m_shoulder = sub1;
        addRequirements(m_shoulder);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shoulder.lowerShoulder();
    }

    @Override
    public void end(boolean interrupted) {
        m_shoulder.stopShoulderMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

}
