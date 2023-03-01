// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ClampSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropCone extends SequentialCommandGroup {
  private static DriveSystem m_driveSystem;
  private static LimelightSystem m_limelightSystem;
  private static ShoulderSystem m_shouldersystem;
  private static ArmSystem m_armsystem;
  private static ClampSystem m_clampsystem;
  /** Creates a new DropCone. */
  public DropCone() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AdjustDirectionToDropCone(m_driveSystem, m_limelightSystem),
      new AdjustShoulderToDropCone(m_shouldersystem, m_limelightSystem),
      new AdjustArmToDropCone(m_armsystem, m_limelightSystem),
      new unclamp(m_clampsystem),
      new armExtendToValue(m_armsystem, 0),
      new RotateShoulderToValue(m_shouldersystem, 0)
    );
  }
}
