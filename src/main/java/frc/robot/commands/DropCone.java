// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ClampSystem;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.switchesSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.DriveSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropCone extends SequentialCommandGroup {
  /** Creates a new DropCone. */
  public DropCone(DriveSystem m_drivesystem, ShoulderSystem m_shouldersystem, ArmSystem m_armsystem, ClampSystem m_clampsystem, switchesSystem m_switchessystem, LimelightSystem m_limelight,int stage) {
    int[] shoulder_rotation = new int[]{67,120,160};
    int[] arm_extension = new int[]{100,6000,8400};
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetLimeLightPipeline(m_limelight, stage-1),
        new RotateShoulderToValue(m_shouldersystem, shoulder_rotation[stage]),
        //new CenterRobot(m_drivesystem, m_limelight),
        new MoveTime(m_drivesystem, .5,.4),
        new armExtendToValue(m_armsystem,m_switchessystem ,arm_extension[stage]),
        new unclamp(m_clampsystem), 
        new armExtendToValue(m_armsystem,m_switchessystem ,0),
        new armExtendToZero(m_armsystem,m_switchessystem),
        new MoveTime(m_drivesystem, -.5,.7),
        new RotateShoulderToValue(m_shouldersystem, 0)
    );
  }
}
