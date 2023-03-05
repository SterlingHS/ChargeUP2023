// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoConeTopBackupToLine extends SequentialCommandGroup {
  /** Creates a new AutoConeBackupToLine. */
  public AutoTwoConeTopBackupToLine(DriveSystem m_drivesystem, ShoulderSystem m_shouldersystem, ArmSystem m_armsystem, ClampSystem m_clampsystem, switchesSystem m_switchsystem, LimelightSystem m_limelightsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Drop first cone
      new DropCone(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem, 2),
      
      // Move to the new cone
      new MoveDistance(m_drivesystem, -2000),
      new MoveTurnAngle(m_drivesystem, 180),
      new MoveDistance(m_drivesystem, 2000),

      // Pick up the new cone
      new PickUp(m_armsystem, m_clampsystem, m_switchsystem),

      // Move bqack to drop off cone
      new MoveTurnAngle(m_drivesystem, 180),
      new MoveDistance(m_drivesystem, 2000),

      // Drop second cone
      new DropCone(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem, 1),
      
      // Move back to line
      new MoveDistance(m_drivesystem, -4000)
    );
  }
}
