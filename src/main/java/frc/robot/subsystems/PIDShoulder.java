// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PIDShoulder extends PIDSubsystem {
  private static Encoder shoulder_encoder;
  private WPI_TalonSRX shoulderMotor;
  /* 
  private final SimpleMotorFeedforward m_shooterFeedforward =
  new SimpleMotorFeedforward(ShooterConstants.kSVolts,
                             ShooterConstants.kVVoltSecondsPerRotation); // this is the feedforward for the shoulder this goes into use output which is automaticallhy called by the PID subsystem  

*\
  /** Creates a new PIDShoulder. */
  public PIDShoulder() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    
    shoulder_encoder = new Encoder(Constants.ENCODER_SHOULDER_A, Constants.ENCODER_SHOULDER_B);
    shoulderMotor = new WPI_TalonSRX(Constants.SHOULDER_MOTOR);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double position = shoulder_encoder.get();
    return position;
  }

  public void lowerShoulder(double speed) {
      shoulderMotor.set(speed); 
  }

  public void raiseShoulder(double speed) {
      shoulderMotor.set(speed); 
  }

  public void stopShoulderMotor() {
      shoulderMotor.stopMotor();
  }

  public int getPosition() {
      return shoulder_encoder.get();
  }

  public void resetEncoder() {
      shoulder_encoder.reset();
  }
}
