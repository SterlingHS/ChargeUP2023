// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class switchesSystem extends SubsystemBase {
  public DigitalInput switchShoulderIn;
  public DigitalInput switchArmIn;
  /** Creates a new switchesSystem. */
  public switchesSystem() {
    switchArmIn = new DigitalInput(Constants.DIO_SWITCH_ARM_IN);
    switchShoulderIn = new DigitalInput( Constants.DIO_SWITCH_SHOULDER_IN);
  }

  public boolean isArmIn() {
    return !switchArmIn.get();
  }

  public boolean isShoulderIn() {
    return switchShoulderIn.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
