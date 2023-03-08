// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ShoulderSystem extends PIDSubsystem {
  /** Creates a new ShoulderSystem. */
  private static Encoder shoulder_encoder;
  private WPI_TalonSRX shoulderMotor1;
  private WPI_TalonSRX shoulderMotor2;
  private switchesSystem m_switchsystem;
  private MotorControllerGroup shoulderMotorGroup;

  // Constructor
  public ShoulderSystem(switchesSystem sub1) {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.PID_SHOULDER_P, Constants.PID_SHOULDER_I, Constants.PID_SHOULDER_D));
    shoulder_encoder = new Encoder(Constants.ENCODER_SHOULDER_A, Constants.ENCODER_SHOULDER_B, false, Encoder.EncodingType.k4X);
    shoulderMotor1 = new WPI_TalonSRX(Constants.SHOULDER_MOTOR_ONE);
    shoulderMotor2 = new WPI_TalonSRX(Constants.SHOULDER_MOTOR_TWO);
    shoulderMotorGroup = new MotorControllerGroup(shoulderMotor1, shoulderMotor2);

    shoulderMotorGroup.setInverted(true);
    m_switchsystem = sub1;
  
    setSetpoint(0);
    getController().setTolerance(5);
  }

  // Calculates the output of the PIDController
  @Override
  public void useOutput(double output, double setpoint) {
    updateShoulderSystem();
    // Use the output here
    rotateShoulder(getController().calculate(getMeasurement(), setpoint));
  }

  // Returns the measurement for the PIDController
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shoulder_encoder.get();
  }

  // Sets the position of the shoulder
  public void setPosition(double position) {
    setSetpoint(position);
  }

  // Returns the position of the shoulder
  public int getPosition() {
    return shoulder_encoder.get();
  }

  // Resets the encoder
  public void resetEncoder() {
    shoulder_encoder.reset();
  }

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

    double setP = getSetPoint();
    if (setP == 0.0 && getPosition() < 20 && m_switchsystem.isShoulderIn() == false) {
      speed = -.08;
    }
    
    // Stop the shoulder from going down if it is at the bottom
    if (m_switchsystem.isShoulderIn() == true && speed < 0) {
      speed = 0;
    }

    /*if (getPosition()==0 && getSetPoint()==0) {
      speed = 0;
    }*/

      SmartDashboard.putNumber("Shoulder Speed", speed);
      shoulderMotorGroup.set(speed);
  }

  // Updates the PID values
  public void updateShoulderSystem() {
    getController().setP(Constants.PID_SHOULDER_P);
    getController().setI(Constants.PID_SHOULDER_I);
    getController().setD(Constants.PID_ARM_D);
  }

  // Stops the shoulder
  public void stop() {
    shoulderMotorGroup.stopMotor();
  } 

  // Returns the error of the PIDController
  public double getError() {
    return getController().getPositionError();
  }

  // Returns the setpoint of the PIDController
  public double getSetPoint() {
    return getController().getSetpoint();
  }

  // Returns the output of the PIDController
  public double getOutput() {
    return getController().calculate(getMeasurement(),getSetPoint());
  }

  // Returns the rate of the encoder
  public double getRate() {
    return shoulder_encoder.getRate();
  }

  // Returns if the PIDController is at the setpoint
  public boolean atSetpoint() {
    return getController().atSetpoint();
  }
}
