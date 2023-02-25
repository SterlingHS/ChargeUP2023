package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDShoulderSystem;
import frc.robot.Constants;

public class lowerShoulder extends CommandBase {
    private final PIDShoulderSystem m_shoulder;
   

    public lowerShoulder(PIDShoulderSystem sub1) {
        m_shoulder = sub1;
        addRequirements(m_shoulder);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shoulder.rotateShoulder(-Constants.SHOULDER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        m_shoulder.stop();
        m_shoulder.setSetpoint(m_shoulder.getPosition());
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