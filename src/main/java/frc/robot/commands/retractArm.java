package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;

public class retractArm extends CommandBase {
    private final ArmSystem m_armsystem;

    public retractArm(ArmSystem sub1) {
        m_armsystem = sub1;
        addRequirements(m_armsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_armsystem.extendArm(-Constants.ARM_SPEED);
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

