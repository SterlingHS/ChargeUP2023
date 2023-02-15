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

public class PIDArm extends PIDSubsystem {
  private static Encoder arm_encoder;
  private WPI_TalonSRX armMotor;
  
  private final SimpleMotorFeedforward m_armFeedforward =
  new SimpleMotorFeedforward(Constants.kSVolts,
                             Constants.kVVoltSecondsPerRotation); // this is the feedforward for the shoulder this goes into use output which is automaticallhy called by the PID subsystem  


  /** Creates a new PIDShoulder. */
  public PIDArm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    
    arm_encoder = new Encoder(Constants.ENCODER_ARM_A, Constants.ENCODER_ARM_B);
    armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    armMotor.setVoltage(output + m_armFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double position = arm_encoder.get();
    return position;
  }

  public void extendArm(double speed) {
    if (speed > 0.4)
    { 
      speed = 0.4;
    }
    if (speed < -0.4)
    { 
      speed = -0.4;
    }
      armMotor.set(speed); 
  }

  public void stopShoulderMotor() {
      armMotor.stopMotor();
  }

  public int getPosition() {
      return arm_encoder.get();
  }

  public void resetEncoder() {
      arm_encoder.reset();
  }
}
