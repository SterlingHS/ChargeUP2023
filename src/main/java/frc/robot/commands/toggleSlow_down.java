// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class toggleSlow_down extends InstantCommand {
  public static DriveSystem m_drivesystem;
  public toggleSlow_down(DriveSystem sub1) {
    m_drivesystem = sub1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivesystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivesystem.toggleSlowdown();
  }
}
