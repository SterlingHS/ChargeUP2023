// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;


//Used for the middle station from our side's perspective



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBoxTopBackupToLine2 extends SequentialCommandGroup {
  /** Creates a new AutoConeBackupToLine. */
  public AutoBoxTopBackupToLine2(DriveSystem m_drivesystem, ShoulderSystem m_shouldersystem, ArmSystem m_armsystem, ClampSystem m_clampsystem, switchesSystem m_switchsystem, LimelightSystem m_limelightsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Drop first cone
      new DropBox(m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_drivesystem, 1),
      // Move backwards to the line
      new MoveTime(m_drivesystem, 0.65, 5),
      //Move forward to platform
      new MoveTime(m_drivesystem, -0.65, 2),
      //Balance on platform
      new  BalanceMiddlePlatform(m_drivesystem)
    );
  }
}
