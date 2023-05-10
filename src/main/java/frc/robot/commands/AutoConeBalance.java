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
public class AutoConeBalance extends SequentialCommandGroup {
  /** Creates a new AutoConeBackupToLine. */
  public AutoConeBalance(DriveSystem m_drivesystem, ShoulderSystem m_shouldersystem, ArmSystem m_armsystem, ClampSystem m_clampsystem, switchesSystem m_switchsystem, LimelightSystem m_limelightsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Drop cone on level one
      new DropConeFirst(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem)
      //Moves 5 metres at constant velocity
      //new MoveDistanceConstant(m_drivesystem, 5, -0.65),
      //Wait to allow robot to get all wheels on ground, obviously
      //new WaitCommand(1.25),
      //new MoveTime(m_drivesystem, 0.65, 1.5),
      //new BalanceMiddlePlatform(m_drivesystem, -1)
    );
  }
}
