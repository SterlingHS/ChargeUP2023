// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSystem;

public class SetLimeLightPipeline extends CommandBase {
  private static LimelightSystem m_limelight;
  private int m_pipeline;
  /** Creates a new SetLimeLightPipeline. */
  public SetLimeLightPipeline(LimelightSystem sub1, int sub2) {
    m_limelight = sub1;
    m_pipeline = sub2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setPipeline(m_pipeline);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
