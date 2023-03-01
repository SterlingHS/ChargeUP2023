// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClampSystem;

public class clamp extends CommandBase {
  /** Creates a new clamp. */
  private ClampSystem m_clampsystem;
  public clamp(ClampSystem sub1) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_clampsystem = sub1;
    addRequirements(m_clampsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clampsystem.closeClamp();
    System.out.println("Clamp closed");
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
