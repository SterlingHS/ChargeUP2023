package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

public class retractArm extends CommandBase {
    private final ArmSystem m_armsysyetem;

    public retractArm(ArmSystem sub1) {
        m_armsysyetem = sub1;
        addRequirements(m_armsysyetem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_armsysyetem.extendArm(-.1);
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

