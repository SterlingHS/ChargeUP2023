package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.switchesSystem;
import frc.robot.Constants;

//Mess File
public class raiseShoulderPID extends CommandBase {
    private final ShoulderSystem m_shoulder;
    private switchesSystem m_switchessystem;

    public raiseShoulderPID(ShoulderSystem sub1, switchesSystem sub2) {
        m_shoulder = sub1;
        m_switchessystem = sub2;
        addRequirements(m_shoulder);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_switchessystem.isArmIn() && m_shoulder.getPosition()<Constants.MAX_SHOULDER_POSITION) {
        m_shoulder.raiseShoulder(-Constants.MAX_SHOULDER_VELOCITY_UP);}
        
    }

    @Override
    public void end(boolean interrupted) {
        m_shoulder.stopShoulderMotor();
        //m_shoulder.setSetpoint(m_shoulder.getPosition());
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