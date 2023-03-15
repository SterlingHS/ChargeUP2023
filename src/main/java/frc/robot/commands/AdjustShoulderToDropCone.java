// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.ShoulderSystem;

public class AdjustShoulderToDropCone extends CommandBase {
  private static ShoulderSystem m_shoulderSystem;
  private static LimelightSystem m_limelightSystem;
  private PIDController pidController = new PIDController(Constants.PID_SHOULDER_P, Constants.PID_SHOULDER_I, Constants.PID_SHOULDER_D);

  /** Creates a new AdjustShoulderToDropCone. */
  public AdjustShoulderToDropCone(ShoulderSystem sub1, LimelightSystem m_limelight) {
    m_shoulderSystem = sub1;
    m_limelightSystem = m_limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    //System.out.println(distance_destination);
    pidController.setSetpoint(0); // Angle Y from limelight to drop cone
    //System.out.println(pidController.getGoal());
    pidController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = m_limelightSystem.getY();
    double output = pidController.calculate(y);
    m_shoulderSystem.rotateShoulder(-output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulderSystem.stop();
    m_shoulderSystem.setSetpoint(m_shoulderSystem.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
