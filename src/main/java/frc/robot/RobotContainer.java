
package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;

//Mess File

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private static SendableChooser<Command> m_chooser;
  private final switchesSystem m_switchsystem = new switchesSystem();
  private final ShoulderSystem m_shouldersystem = new ShoulderSystem(m_switchsystem);
  private final DriveSystem m_drivesystem = new DriveSystem();
  private final ArmSystem m_armsystem = new ArmSystem(m_switchsystem);
  private final ClampSystem m_clampsystem = new ClampSystem();
  private final LimelightSystem m_limelightsystem = new LimelightSystem();
  
  // Joysticks
  private final XboxController driverController = new XboxController(Constants.MAIN_JOYDRIVER_USB_PORT);
  private final XboxController codriverController = new XboxController(Constants.CO_JOYDRIVER_USB_PORT);


  
  // A chooser for autonomous commands
  


  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {

    m_shouldersystem.enable();
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Configure the button bindings
    
    
    //configure the limit switches
    configureLimitSwitches();
    m_chooser.setDefaultOption("Auto Box One", new AutoBoxTopBackupToLine3(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    m_chooser.addOption("Movetime Test",new MoveTime(m_drivesystem, 0.5,1000));
    m_chooser.addOption("Auto Box Two", new AutoBoxTopBackupToLine2(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));

    SmartDashboard.putData(m_chooser);
    

    SmartDashboard.putNumber("Shoulder P", Constants.PID_SHOULDER_P);
    SmartDashboard.putNumber("Shoulder I", Constants.PID_SHOULDER_I);
    SmartDashboard.putNumber("Shoulder D", Constants.PID_SHOULDER_D);
    SmartDashboard.putBoolean("Slowdown On", false);

    SmartDashboard.putNumber("Shoulder Speed", 0);
    SmartDashboard.putNumber("Output", 0);
    SmartDashboard.putNumber("Feedforward", 0);
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
  public void configureButtonBindings() {

    //******************Driver Controller****************

    // **************************************************
    // ARM SYSTEM

    m_drivesystem.setDefaultCommand(new Drive( m_drivesystem, driverController::getLeftY, driverController::getRightX) ); 

    //Button To extend Arm -- Uses X Button
    new JoystickButton(driverController, XboxController.Button.kB.value).whileTrue(new extendArm(m_armsystem)); 
    //Button To extend Arm -- Uses B Button
    new JoystickButton(driverController, XboxController.Button.kX.value).whileTrue(new retractArm(m_armsystem)); 

    // Button to extend arm to a certain value -- Uses Right Bumper
    //new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new armExtendToValue(m_armsystem, 0));
    //new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).onTrue(new armExtendToValue(m_armsystem, 5000));


    // **************************************************
    // SHOULDER SYSTEM

    //Button To raise shoulder -- Uses A Button
    new JoystickButton(driverController, XboxController.Button.kA.value).whileTrue(new lowerShoulder(m_shouldersystem, m_switchsystem));
    //Button To lower shoulder -- Uses Y Button
    new JoystickButton(driverController, XboxController.Button.kY.value).whileTrue(new raiseShoulder(m_shouldersystem, m_switchsystem));
    final POVButton encoderResetBt = new POVButton(driverController, Constants.POV_LEFT);
    encoderResetBt.onTrue(new shoulderResetEncoder(m_shouldersystem));
    
    // **************************************************
    // Automatic droppers

    // Button to drop box on 2nd level
    final TriggerR2Button BoxTwoBt = new TriggerR2Button(driverController);
    new Trigger(BoxTwoBt::get).onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem, m_drivesystem, 2));

    // Button to drop cone on 1st level
    //new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new DropCone(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem, 1));
  
    // Button to drop box on 1st level
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem,m_drivesystem, 1));
 
    // Button to drop to floor level
    final POVButton DropFloorBt = new POVButton(driverController,Constants.POV_UP); 
    DropFloorBt.onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem, m_drivesystem, 0));

    // Button to pick up object
    final POVButton PickUpBt = new POVButton(driverController,Constants.POV_DOWN); 
    PickUpBt.onTrue(new PickUp(m_armsystem, m_clampsystem,m_switchsystem));

    final POVButton RaiseToShelf = new POVButton(driverController, Constants.POV_RIGHT);
    RaiseToShelf.onTrue(new RotateShoulderToValue(m_shouldersystem, 13));

    final POVButton PickUpOut = new POVButton(codriverController, Constants.POV_DOWN);
    PickUpOut.onTrue(new PickUpOutside(m_armsystem, m_clampsystem, m_switchsystem, m_shouldersystem));

    //Button to move forward
    /*final POVButton GoForwardBt = new POVButton(driverController,Constants.POV_RIGHT);
    GoForwardBt.onTrue(new MoveDistance(m_drivesystem, 0.25));*/
  
    // Button to drop cone on 2nd level
    final TriggerL2Button ConeTwoBt = new TriggerL2Button(driverController);
    new Trigger(ConeTwoBt::get).onTrue(new DropCone(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem, 2));
  
    

    // **************************************************
    // CLAMP SYSTEM

    // Button to clamp -- Uses Right Stick Button
    new JoystickButton(driverController, XboxController.Button.kStart.value).onTrue(new clamp(m_clampsystem));
    // Button to unclamp -- Uses Left Stick Button
    new JoystickButton(driverController, XboxController.Button.kBack.value).onTrue(new unclamp(m_clampsystem));


    /*
    *
    End of Main driver controller mapping; beginning of co-driver controller mappings 
    * 
    */

    //new JoystickButton(codriverController, XboxController.Button.kStart.value).whileTrue(new updateShoulderPID(m_shouldersystem));

    
    
  }
  private void configureLimitSwitches(){
      // new Trigger(m_switchsystem.switchArmIn::get).whileTrue(new armResetEncoder(m_armsystem)); // command to reset encoder, called when limit switch is pressed
      // new Trigger(m_switchsystem.switchShoulderIn::get).onTrue(new shoulderResetEncoder(m_shouldersystem));
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
    return new AutoBoxTopBackupToLine2(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem);
    //return m_chooser.getSelected();
  }

  public void update_smartboard(){
        // SmartDashboard.putBoolean("DIO 9", drivesystem.state_DIO9());
        // SmartDashboard.putNumber("Right Pulse", m_drivesystem.read_pulse_right_encoder());
        // SmartDashboard.putNumber("Left Pulse", m_drivesystem.read_pulse_left_encoder());
        // SmartDashboard.putNumber("Right Distance", m_drivesystem.read_distance_right_encoder());
        // SmartDashboard.putNumber("Left Distance", m_drivesystem.read_distance_left_encoder());
        // SmartDashboard.putNumber("Velocity", m_drivesystem.read_velocity_encoder());
        // SmartDashboard.putNumber("Angle", m_drivesystem.getAngle360());
        // SmartDashboard.putNumber("Pitch", m_drivesystem.getPitch());
        // SmartDashboard.putNumber("Roll", m_drivesystem.getRoll());
        // SmartDashboard.putNumber("Compass Heading", m_drivesystem.getCompassHeading());
        // SmartDashboard.putNumber("Fused Heading", m_drivesystem.getFusedHeading());
        // SmartDashboard.putNumber("Linear World Accel X", m_drivesystem.getLinearWorldAccelX());
        // SmartDashboard.putNumber("Linear World Accel Y", m_drivesystem.getLinearWorldAccelY());
        // SmartDashboard.putNumber("Linear World Accel Z", m_drivesystem.getLinearWorldAccelZ());
        // SmartDashboard.getNumber("P", Constants.PID_ARM_P);
        // SmartDashboard.getNumber("I", Constants.PID_ARM_I);
        // SmartDashboard.getNumber("D", Constants.PID_ARM_D);
        SmartDashboard.putNumber("Shoulder Position", m_shouldersystem.getPosition());
        SmartDashboard.putNumber("Arm Position", m_armsystem.getPosition());
        SmartDashboard.putBoolean("Arm In", m_switchsystem.isArmIn());
        SmartDashboard.putBoolean("Shoulder In", m_switchsystem.isShoulderIn());
        SmartDashboard.putNumber("Shoulder Error", m_shouldersystem.getError());
        SmartDashboard.putNumber("Shoulder Setpoint", m_shouldersystem.getSetpoint());
        SmartDashboard.putNumber("Shoulder Output", m_shouldersystem.getOutput());
        SmartDashboard.putNumber("Shoulder Rate", m_shouldersystem.getRate());
        SmartDashboard.putNumber("Left Count", m_drivesystem.getLeftEncoder());
        SmartDashboard.putNumber("Right Count", m_drivesystem.getRightEncoder());
        SmartDashboard.putNumber("Left Distance", m_drivesystem.getLeftDistance());
        SmartDashboard.putNumber("Right Distance", m_drivesystem.getRightDistance());
        SmartDashboard.putNumber("Average Distance", m_drivesystem.getDistance());
        // SmartDashboard.putString("Auto Command Choser", m_chooser.getSelected().getName());
        SmartDashboard.putNumber("Shoulder Volts 1", m_shouldersystem.getShoulderMotorOneVoltage());
        SmartDashboard.putNumber("Shoulder Volts 2", m_shouldersystem.getShoulderMotorTwoVoltage());
        SmartDashboard.putNumber("Pitch", m_drivesystem.getPitch());
        SmartDashboard.putNumber("Roll", m_drivesystem.getRoll());
        SmartDashboard.putNumber("Yaw", m_drivesystem.getYaw());
        
        
        Constants.PID_SHOULDER_P = SmartDashboard.getNumber("Shoulder P", 0.04);
        Constants.PID_SHOULDER_I = SmartDashboard.getNumber("Shoulder I", 0);
        Constants.PID_SHOULDER_D = SmartDashboard.getNumber("Shoulder D", 0);
        m_drivesystem.putSlowdown(SmartDashboard.getBoolean("Slowdown On", false));
  }
}
