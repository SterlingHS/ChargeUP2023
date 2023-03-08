package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.switchesSystem;
import frc.robot.Constants;

public class lowerShoulder extends CommandBase {
    private final ShoulderSystem m_shoulder;
    private switchesSystem m_switchessystem;
   

    public lowerShoulder(ShoulderSystem sub1, switchesSystem sub2) {
        m_shoulder = sub1;
        m_switchessystem = sub2;
        addRequirements(m_shoulder);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_switchessystem.isArmIn()) {
            m_shoulder.rotateShoulder(-Constants.SHOULDER_SPEED);
            //System.out.println("Lowering Shoulder");
        }
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