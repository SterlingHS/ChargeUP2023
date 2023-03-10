// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShoulderSystemTrap extends SubsystemBase {
  
  private static Encoder shoulder_encoder;
  private WPI_TalonSRX shoulderMotor1;
  private WPI_TalonSRX shoulderMotor2;
  private switchesSystem m_switchsystem;
  private MotorControllerGroup shoulderMotorGroup;
  private final ArmFeedforward m_feedforward;
  private final ProfiledPIDController m_pidController;

  private double kS = .88916;
  private double kG = .63697;
  private double kV = 0.001915;
  private double kA = 0.003268;

  private double MaxVelocity = 0.1;
  private double MaxAcceleration = 0.1;
  private TrapezoidProfile.State setpoint;

  /** Creates a new ShoulderSystemTrapTest. */
  public ShoulderSystemTrap(switchesSystem sub1) {
    shoulder_encoder = new Encoder(Constants.ENCODER_SHOULDER_A, Constants.ENCODER_SHOULDER_B, false, Encoder.EncodingType.k4X);
    shoulderMotor1 = new WPI_TalonSRX(Constants.SHOULDER_MOTOR_ONE);
    shoulderMotor2 = new WPI_TalonSRX(Constants.SHOULDER_MOTOR_TWO);
    shoulderMotorGroup = new MotorControllerGroup(shoulderMotor1, shoulderMotor2);

    shoulderMotorGroup.setInverted(true);
    m_switchsystem = sub1;

    m_pidController = new ProfiledPIDController(Constants.PID_SHOULDER_P, Constants.PID_SHOULDER_I, Constants.PID_SHOULDER_D, new TrapezoidProfile.Constraints(MaxVelocity, MaxAcceleration));
    m_pidController.setTolerance(3*Math.PI/180); // 3 degrees tolerance
    setpoint = new TrapezoidProfile.State(calculateAngle(0), 0);
    m_pidController.setGoal(setpoint);
    m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_switchsystem.isShoulderIn() == true) {
      resetEncoder();
    }
    rotateToAngle(m_pidController.getGoal().position);
  }

  // ********** Motor Methods ********** //

  // Rotates the shoulder
  public void rotateShoulder(double speed) {
    // Limit the speed of the shoulder going up
    if (speed > Constants.MAX_SHOULDER_VELOCITY_UP) {
      speed = Constants.MAX_SHOULDER_VELOCITY_UP;
    }
    // Limit the speed of the shoulder going down
    if (speed < -Constants.MAX_SHOULDER_VELOCITY) {
      speed = -Constants.MAX_SHOULDER_VELOCITY;
    }    
    
    // Stop the shoulder from going down if it is at the bottom
    if (m_switchsystem.isShoulderIn() == true && speed < 0) {
      speed = 0;
    }

    
    SmartDashboard.putNumber("Shoulder Speed", speed);
    shoulderMotorGroup.set(speed);
  }

  // Stops the shoulder
  public void stop() {
  shoulderMotorGroup.stopMotor();
} 

  // Rotates the shoulder to a specific angle
  public void rotateToAngle(double position) {
    double angleDestination = calculateAngle(position);
    double feedforward = m_feedforward.calculate(angleDestination, getAngularRate());
    double output = m_pidController.calculate(getAngle(), angleDestination);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Feedforward", feedforward);
    rotateShoulder(output); //+feedforward);
  }
  
  // ********** PID Methods ********** //

  // Set the setpoint position only
  public void setSetpoint(double position) {
    setpoint = new TrapezoidProfile.State(calculateAngle(position), 0);
    m_pidController.setGoal(setpoint);
    System.out.println("Setpoint: " + position);
  }

  // Gets the setpoint of the PID controller
  public TrapezoidProfile.State getSetpointTrap() {
    return m_pidController.getGoal(); // Return class TrapezoidProfile.State
  }

  // Gets the setpoint of the PID controller
  public double getSetpoint() {
    return m_pidController.getGoal().position; // Return class TrapezoidProfile.State
  }

  // Returns the error of the PID controller
  public double getError() {
    return m_pidController.getPositionError();
  }

  // Updates the PID constants
  public void updatePIDConstants(double p, double i, double d) {
    m_pidController.setPID(p, i, d);
  }

  // Updates the feedforward constants
  public void updateFeedforwardConstants(double s, double v, double a) {
    kS = s;
    kV = v;
    kA = a;
  }

  // Get Output of PID controller
  public double getOutput() {
    return m_pidController.calculate(getAngle(), getSetpoint());
  }


  public boolean atSetpoint() {
    return m_pidController.atSetpoint(); // Should we use atGoal?
  }

  // ********** Encoder Methods ********** //

  // Returns the position of the shoulder
  public int getPosition() {
    return shoulder_encoder.get();
  }

  // Resets the encoder
  public void resetEncoder() {
    shoulder_encoder.reset();
  }

  // Returns the rate of the encoder
  public double getRate() {
    return shoulder_encoder.getRate();
  }

  // Calculates the angle in radians of the shoulder with respect of the horizontal
  public double getAngle() {
    return calculateAngle(getPosition());
  }

  public double calculateAngle(double position) {
    double posMax = 170; // Ticks of the encoder when the shoulder is horizontal
    double posMin = 0;     // Ticks of the encoder when the shoulder is vertical
    double angleMax = 41;
    double angleMin = -79;
    double angle = ((angleMax-angleMin)/posMax)*position + angleMin;
    //double angle = (pos-posHorizontal) * Math.PI / (2*(posHorizontal-posVertical));
    return angle;
  }

  // Calculates the angular rate of the shoulder
  public double getAngularRate() {
    double rate = getRate();
    //double posMax = 170; // Ticks of the encoder when the shoulder is horizontal
    //double posMin = 0;     // Ticks of the encoder when the shoulder is vertical
    double angularRate = calculateAngle(rate)+79;

    return angularRate;
  }
}
