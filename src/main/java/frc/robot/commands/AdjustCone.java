// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.LimelightSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustCone extends ParallelCommandGroup {
  /** Creates a new AdjustCone. */
  public AdjustCone(DriveSystem m_drivesystem, ShoulderSystem m_shouldersystem, LimelightSystem m_limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AdjustShoulderToDropCone(m_shouldersystem, m_limelight),
        new AdjustDirectionToDropCone(m_drivesystem, m_limelight)
    );
  }
}
