// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerL2Button extends CommandBase {
  private PS4Controller driverController;
  /** Creates a new TriggerL2Button. */
  public TriggerL2Button(PS4Controller sub1) {
    driverController = sub1;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public boolean get() {
    return driverController.getRawAxis(2) > 0.5;
  }
}
