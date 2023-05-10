// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;


//Middle



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoConeBalanceNoLine extends SequentialCommandGroup {
  /** Creates a new AutoConeBackupToLine. */
  public AutoConeBalanceNoLine(DriveSystem m_drivesystem, ShoulderSystem m_shouldersystem, ArmSystem m_armsystem, ClampSystem m_clampsystem, switchesSystem m_switchsystem, LimelightSystem m_limelightsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Drop first box
      new DropConeFirst(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem),
      //Turn Slightly
      // Move backwards to the line
      //new MoveTime(m_drivesystem, 0.65, 5),
      // new MoveDistance(m_drivesystem, -1.542),
      //Move backward to platform
      //new MoveDistanceConstant(m_drivesystem, 5, -0.65),
      // new MoveDistance(m_drivesystem, -0.9),
      //Balance on platform
      //new WaitCommand(1.25),
      new MoveTime(m_drivesystem, -.6, 0.5),
      //new WaitCommand(0.5),
      new TurnRobotAngleRight(m_drivesystem, 180),
      //new MoveTime(m_drivesystem, .6, 2),
      //new BalanceMiddlePlatform(m_drivesystem, -1)
      new MoveTime(m_drivesystem, 0.63, 2),
      new  BalanceMiddlePlatform(m_drivesystem, -1)
    );
  }
}
