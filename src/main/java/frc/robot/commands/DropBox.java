// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ClampSystem;
import frc.robot.subsystems.ShoulderSystem;
import frc.robot.subsystems.switchesSystem;
import frc.robot.subsystems.DriveSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropBox extends SequentialCommandGroup {
  /** Creates a new DropBox. */
  public DropBox(ShoulderSystem m_shouldersystem, ArmSystem m_armsystem, ClampSystem m_clampsystem, switchesSystem m_switchessystem,DriveSystem m_drivesystem, int stage) {
    int[] shoulder_rotation = new int[]{300,577,670};
    int[] arm_extension = new int[]{100,6000,12000};
    //List<Double> shoulder_rotation = new ArrayList<Double>();
    //shoulder_rotation = Arrays.asList(20,50,80);
    //List<Double> arm_extension = Arrays.asList(300,400,600);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RotateShoulderToValue(m_shouldersystem, shoulder_rotation[stage]),
        new MoveTime(m_drivesystem, .5,.4),
        //new MoveDistance(m_drivesystem, .5),
        new armExtendToValue(m_armsystem,m_switchessystem ,arm_extension[stage]),
        new unclamp(m_clampsystem), 
        new WaitCommand(.25), 
        new armExtendToValue(m_armsystem, m_switchessystem, 0),
        new armExtendToZero(m_armsystem, m_switchessystem),
        new MoveTime(m_drivesystem, -0.4,1),
        //new MoveDistance(m_drivesystem, -1),
        new RotateShoulderToValue(m_shouldersystem, 0)
    );
  }
}
