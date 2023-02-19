// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;

public class armExtendTo10000 extends CommandBase {
  private static ArmSystem m_armsystem;
  private static PIDController m_pidController;

  /** Creates a new armExtendToValue. */
  public armExtendTo10000(ArmSystem sub1) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armsystem = sub1;
    System.out.println("Initialization of destination"+10000);
    addRequirements(m_armsystem);
    m_pidController = new PIDController(Constants.PID_ARM_P, Constants.PID_ARM_I, Constants.PID_ARM_D);
    m_pidController.setSetpoint(10000);
    m_pidController.setTolerance(Constants.kTurnToleranceDeg, Constants.kTurnRateToleranceDegPerS);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_pidController.calculate(m_armsystem.getPosition(), 10000);
    m_armsystem.extendArm(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armsystem.stopArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }

  // Commands to check error and output
  public static double getError() {
    return m_pidController.getPositionError();
  }
  
  public static double getOutput() {
    return m_pidController.calculate(m_armsystem.getPosition());
  }

  /*public static void printOutput() {
    System.out.println("Output: " + getOutput());
  }

  public static void printError() {
  }

  public static void printDestination(double destination) {
    System.out.print(destination);*/
  }



