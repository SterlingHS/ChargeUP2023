// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.LimelightSystem;

public class AdjustArmToDropCone extends CommandBase {
  private static ArmSystem m_armSystem;
  private static LimelightSystem m_limelightSystem;
  /** Creates a new AdjustArmToDropCone. */
  public AdjustArmToDropCone(ArmSystem sub1, LimelightSystem sub2) {
    m_armSystem = sub1;
    m_limelightSystem = sub2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSystem);
    addRequirements(m_limelightSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double area = m_limelightSystem.getArea();
    if (area > 0.1) {
      m_armSystem.extendArm(0.2);
    } else if (area < 0.1) {
      m_armSystem.extendArm(-0.2);
    } else {
      m_armSystem.stopArmMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSystem.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double area = m_limelightSystem.getArea();
    if (area > 0.09 && area < 0.11) {
      return true;
    }
    return false;
  }
}
