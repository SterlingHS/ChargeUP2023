// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDarmExtendToValue extends PIDCommand {
  /** Creates a new PIDarmExtendToValue. */
  public PIDarmExtendToValue(double destination, ArmSystem m_ArmSystem) {
    super(
        // The controller that the command will use
        new PIDController(Constants.PID_ARM_P, Constants.PID_ARM_I, Constants.PID_ARM_D),
        // This should return the measurement
        m_ArmSystem::getPosition,
        // This should return the setpoint (can also be a constant)
        destination,
        // This uses the output
        output -> m_ArmSystem.extendArm(output)
          // Use the output here
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

     // Set the controller to be continuous (because it is an angle controller)
     getController().enableContinuousInput(Constants.MIN_ARM_POSITION, Constants.MAX_ARM_POSITION);

     // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.kTurnToleranceDeg, Constants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
    }
}
