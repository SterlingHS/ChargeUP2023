package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;

public class extendArm extends CommandBase {
    private final ArmSystem m_armsysyetem;

    public extendArm(ArmSystem sub1) {
        m_armsysyetem = sub1;
        addRequirements(m_armsysyetem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_armsysyetem.getPosition()<Constants.MAX_ARM_POSITION){
        m_armsysyetem.extendArm(Constants.ARM_SPEED);
        System.out.println(m_armsysyetem.getPosition());
    }

    }

    @Override
    public void end(boolean interrupted) {
        m_armsysyetem.stopArmMotor();
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
