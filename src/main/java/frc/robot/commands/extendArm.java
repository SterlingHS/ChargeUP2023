package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;

public class extendArm extends CommandBase {
    private final ArmSystem m_armsystem;

    public extendArm(ArmSystem sub1) {
        m_armsystem = sub1;
        addRequirements(m_armsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_armsystem.getPosition()<Constants.MAX_ARM_POSITION){
            m_armsystem.extendArm(Constants.ARM_SPEED);
            System.out.println(m_armsystem.getPosition());
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_armsystem.stopArmMotor();
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
