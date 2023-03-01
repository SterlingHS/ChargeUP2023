// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LimelightSystem;

public class AdjustDirectionToDropCone extends CommandBase {
  private static DriveSystem m_driveSystem;
  private static LimelightSystem m_limelightSystem;

  /** Creates a new AdjustDirectionToDropCone. */
  public AdjustDirectionToDropCone(DriveSystem sub1, LimelightSystem sub2) {
    m_driveSystem = sub1;
    m_limelightSystem = sub2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSystem);
    addRequirements(m_limelightSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_limelightSystem.getX();
    if (x > 0) {
      m_driveSystem.turnLeft();
    } else if (x < 0) {
      m_driveSystem.turnRight();
    } else {
      m_driveSystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double x = m_limelightSystem.getX();
    if (x > -10 && x < 10) {
      return true;
    }
    return false;
  }
}
