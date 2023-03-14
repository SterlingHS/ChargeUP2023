// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.ShoulderSystem;

public class AdjustShoulderToDropCone extends CommandBase {
  private static ShoulderSystem m_shoulderSystem;
  private static LimelightSystem m_limelightSystem;
  /** Creates a new AdjustShoulderToDropCone. */
  public AdjustShoulderToDropCone(ShoulderSystem sub1, LimelightSystem m_limelight) {
    m_shoulderSystem = sub1;
    m_limelightSystem = m_limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    double y = m_limelightSystem.getY();
    if (y > 5) {
      speed = -.2;
    }
    else if (y < -5) {
      speed = .2;
    } else {
      speed = 0;
    }
    m_shoulderSystem.rotateShoulder(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulderSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double y = m_limelightSystem.getY();
    if (y > -5 && y < 5) {
      return true;
    }
    return false;
  }
}
