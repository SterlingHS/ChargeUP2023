package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.switchesSystem;
import frc.robot.Constants;

public class raiseShoulder extends CommandBase {
    private final ShoulderSystem m_shoulder;
    private switchesSystem m_switchessystem;

    public raiseShoulder(ShoulderSystem sub1, switchesSystem sub2) {
        m_shoulder = sub1;
        m_switchessystem = sub2;
        addRequirements(m_shoulder);
    }
    
    @Override
    public void initialize() {
        m_shoulder.disable();
    }

    @Override
    public void execute() {
        //System.out.println("UP SHOULDER");
        if (m_switchessystem.isArmIn() && m_shoulder.getPosition()<Constants.MAX_SHOULDER_POSITION) {
            m_shoulder.rotateShoulderForce(Constants.MAX_SHOULDER_VELOCITY_UP);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shoulder.stop();
        m_shoulder.enable(); 
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