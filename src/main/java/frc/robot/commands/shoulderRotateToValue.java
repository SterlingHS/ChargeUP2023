// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShoulderSystem;

public class shoulderRotateToValue extends CommandBase {
  private static ShoulderSystem m_shoulder_system;
  private PIDController m_pidController;
  /** Creates a new armExtendToValue. */
  public shoulderRotateToValue(ShoulderSystem sub1, double dest) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder_system = sub1;
    addRequirements(m_shoulder_system);
    m_pidController = new PIDController(Constants.PID_ARM_P, Constants.PID_ARM_I, Constants.PID_ARM_D);
    m_pidController.setSetpoint(dest);
    m_pidController.setTolerance(Constants.kTurnToleranceDeg, Constants.kTurnRateToleranceDegPerS);
    //m_pidController.enableContinuousInput(Constants.MIN_ARM_POSITION, Constants.MAX_ARM_POSITION);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_pidController.calculate(m_shoulder_system.getPosition(), m_pidController.getSetpoint());
    m_shoulder_system.raiseShoulder(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulder_system.stopShoulderMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }

  // Commands to check error and output
  public double getError() {
    return m_pidController.getPositionError();
  }
  


  /*public static void printOutput() {
    System.out.println("Output: " + getOutput());
  }

  public static void printError() {
  }

  public static void printDestination(double destination) {
    System.out.print(destination);*/
  }


