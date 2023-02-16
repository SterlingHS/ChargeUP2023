
package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final ShoulderSystem m_shouldersystem = new ShoulderSystem();
  private final DriveSystem m_drivesystem = new DriveSystem();
  private final ArmSystem m_armsystem = new ArmSystem();
  
  // Joysticks
  private final XboxController driverController = new XboxController(Constants.JOYDRIVER_USB_PORT);

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_drivesystem.setDefaultCommand(new Drive( m_drivesystem, driverController::getRightX, driverController::getLeftY) ); 
    

    m_chooser.setDefaultOption("string",new MoveTime(m_drivesystem, 0.5,1000));
  }

  /*public static RobotContainer getInstance() {
    return m_robotContainer;
  }*/

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        //Button To extend Arm -- Uses X Button
    final JoystickButton extendArmbt = new JoystickButton(driverController, XboxController.Button.kB.value);
    extendArmbt.whileTrue(new extendArm(m_armsystem)); //whileHeld method is deprecated; change later
    //Button To extend Arm -- Uses B Button
    final JoystickButton retractArmbt = new JoystickButton(driverController, XboxController.Button.kX.value);
    retractArmbt.whileTrue(new retractArm(m_armsystem)); //whileHeld method is deprecated; change later

        // Stabilize robot to drive straight with gyro when left bumper is held
    //new JoystickButton(driverController, XboxController.Button.kRightBumper.value).whileTrue(new PIDarmExtendToValue(600, m_armsystem));
    //new JoystickButton(driverController, XboxController.Button.kA.value).whileTrue(new PIDshoulderRaiseToValue(50, m_shouldersystem));
    //new JoystickButton(driverController, XboxController.Button.kY.value).whileTrue(new PIDshoulderRaiseToValue(500, m_shouldersystem));


  }
  

  public XboxController getDriverController() {
    return driverController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    
    return m_chooser.getSelected();
  }

 public void update_smartboard(){
        // SmartDashboard.putBoolean("DIO 9", drivesystem.state_DIO9());
        SmartDashboard.putNumber("Right Pulse", m_drivesystem.read_pulse_right_encoder());
        SmartDashboard.putNumber("Left Pulse", m_drivesystem.read_pulse_left_encoder());
        SmartDashboard.putNumber("Right Distance", m_drivesystem.read_distance_right_encoder());
        SmartDashboard.putNumber("Left Distance", m_drivesystem.read_distance_left_encoder());
        SmartDashboard.putNumber("Velocity", m_drivesystem.read_velocity_encoder());
        SmartDashboard.putNumber("Angle", m_drivesystem.getAngle360());
        SmartDashboard.putNumber("Pitch", m_drivesystem.getPitch());
        SmartDashboard.putNumber("Roll", m_drivesystem.getRoll());
        SmartDashboard.putNumber("Compass Heading", m_drivesystem.getCompassHeading());
        SmartDashboard.putNumber("Fused Heading", m_drivesystem.getFusedHeading());
        SmartDashboard.putNumber("Linear World Accel X", m_drivesystem.getLinearWorldAccelX());
        SmartDashboard.putNumber("Linear World Accel Y", m_drivesystem.getLinearWorldAccelY());
        SmartDashboard.putNumber("Linear World Accel Z", m_drivesystem.getLinearWorldAccelZ());
        SmartDashboard.putNumber("Shoulder Position", m_shouldersystem.getPosition());
        SmartDashboard.putNumber("Arm Position", m_armsystem.getPosition());
        SmartDashboard.putBoolean("Arm In", m_armsystem.isArmIn());

}
}
