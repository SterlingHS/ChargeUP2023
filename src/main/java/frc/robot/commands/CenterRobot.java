// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.DriveSystem;

public class CenterRobot extends CommandBase {
  private static DriveSystem m_drivesystem;
  private static LimelightSystem m_limelight;
  /** Creates a new CenterRobot. */
  public CenterRobot( DriveSystem sub1, LimelightSystem sub2) {
    m_drivesystem = sub1;
    m_limelight = sub2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivesystem, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.getX() > 0){
      m_drivesystem.turnLeft();
    }else if(m_limelight.getX() < 0){
      m_drivesystem.turnRight();
    }else{
      m_drivesystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.getX() < 5 && m_limelight.getX() > -5;
  }
}
