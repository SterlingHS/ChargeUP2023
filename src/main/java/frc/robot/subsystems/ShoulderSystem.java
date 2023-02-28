// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;



//Formerly called "PIDShoulderSystem"

public class ShoulderSystem extends PIDSubsystem {
  /** Creates a new ShoulderSystem. */
  private static Encoder shoulder_encoder;
  private WPI_TalonSRX shoulderMotor1;
  private WPI_TalonSRX shoulderMotor2;
  private switchesSystem m_switchsystem;
  private MotorControllerGroup shoulderMotorGroup;




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
  
    //shoulderMotor.setInverted(true);
    setSetpoint(0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    updateShoulderSystem();
    // Use the output here
    rotateShoulder(getController().calculate(getMeasurement(), setpoint));
    //System.out.println("Output: " + output + "    Setpoint: " + setpoint);
    //System.out.println("Calculate: " + getController().calculate(getMeasurement(), setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shoulder_encoder.get();
  }

  public void setPosition(double position) {
    setSetpoint(position);
  }

  public int getPosition() {
    if (m_switchsystem.isShoulderIn() == true) {
      resetEncoder();
    }
    return shoulder_encoder.get();
  }

  public void resetEncoder() {
    shoulder_encoder.reset();
  }

  public void rotateShoulder(double speed) {
    if (speed > Constants.MAX_SHOULDER_VELOCITY) {
      speed = Constants.MAX_SHOULDER_VELOCITY;
    }
    if (speed < -Constants.MAX_SHOULDER_VELOCITY) {
      speed = -Constants.MAX_SHOULDER_VELOCITY;
    }
    if (m_switchsystem.isShoulderIn() == true && speed < 0) {
      speed = 0;
    }
    double setP = getSetPoint();
    if (setP == 0.0 && getPosition() < 15 && m_switchsystem.isShoulderIn() == false) {
      speed = -.02;
      System.out.println(speed);
    }
    if (setP == 0.0 && getPosition() < 3 && m_switchsystem.isShoulderIn() == false) {
      speed = -.01;
      System.out.println(speed);
    }
    //System.out.println(speed);
    shoulderMotorGroup.set(speed);
  }

  public void updateShoulderSystem() {
    getController().setP(Constants.PID_SHOULDER_P);
    getController().setI(Constants.PID_SHOULDER_I);
    getController().setD(Constants.PID_ARM_D);
  }

  public void stop() {
    shoulderMotorGroup.stopMotor();;
  } 

  public double getError() {
    return getController().getPositionError();
  }

  public double getSetPoint() {
    return getController().getSetpoint();
  }

  public double getOutput() {
    return getController().calculate(getMeasurement(),getSetPoint());
  }

  public double getRate() {
    return shoulder_encoder.getRate();
  }

}
