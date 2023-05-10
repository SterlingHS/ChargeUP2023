
package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller;
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
  private final PS4Controller driverController = new PS4Controller(Constants.MAIN_JOYDRIVER_USB_PORT);
  private final PS4Controller codriverController = new PS4Controller(Constants.CO_JOYDRIVER_USB_PORT);

  // private final PS4Controller driverController = new PS4Controller(Constants.MAIN_JOYDRIVER_USB_PORT);
  // private final PS4Controller codriverController = new PS4Controller(Constants.CO_JOYDRIVER_USB_PORT);


  
  // A chooser for autonomous commands
  


  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  public RobotContainer() {

    m_shouldersystem.enable();
    // Configure the button bindings
    
    
    //configure the limit switches
    configureLimitSwitches();

    m_chooser = new SendableChooser<>();
    //m_chooser.setDefaultOption("DropCone", new DropConeFirst(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
   // m_chooser.addOption("Auto Box One", new AutoBoxTopBackupToLine3(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    //m_chooser.addOption("Movetime Test",new MoveTime(m_drivesystem, 0.5,1000));
    //m_chooser.addOption("Auto Box Two", new AutoBoxTopBackupToLine2(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    //m_chooser.setDefaultOption("Drop Cone Middle", new AutoConeBalance(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    m_chooser.addOption("Cone Middle", new AutoConeBalance(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    //m_chooser.addOption("Red Middle", new AutoConeBalance(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
   // m_chooser.addOption("Drop Cone Right", new AutoConeTopBackupToLine1(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    //m_chooser.addOption("Drop Cone Left", new AutoConeTopBackupToLine2(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
   //m_chooser.addOption("Blue Long", new AutoConeTopBackupToLine1(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
    //m_chooser.addOption("Red Short", new AutoConeTopBackupToLine1(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
   // m_chooser.addOption("Red Long", new AutoConeTopBackupToLine2(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
   m_chooser.setDefaultOption("Drop Cone Sides First", new AutoConeSidesFirst(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
   //m_chooser.addOption("Test Turn Positive", new TurnRobotAngle(m_drivesystem, 90));
   //m_chooser.addOption("Test Turn Negative", new TurnRobotAngle(m_drivesystem, -90));
   //m_chooser.addOption("Test Slowdown", new toggleSlow_down(m_drivesystem));
    
    

    // m_drivesystem.calibrateGyro();
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
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {

    //******************Driver Controller****************

    // **************************************************
    // ARM SYSTEM

    m_drivesystem.setDefaultCommand(new Drive( m_drivesystem, driverController::getLeftY, driverController::getRightX) ); 

    //Button To extend Arm -- Uses X Button
    new JoystickButton(driverController, PS4Controller.Button.kCircle.value).whileTrue(new extendArm(m_armsystem)); 
    //Button To extend Arm -- Uses B Button
    new JoystickButton(driverController, PS4Controller.Button.kSquare.value).whileTrue(new retractArm(m_armsystem)); 

    // Button to extend arm to a certain value -- Uses Right Bumper
    //new JoystickButton(driverController, PS4Controller.Button.kRightBumper.value).onTrue(new armExtendToValue(m_armsystem, 0));
    //new JoystickButton(driverController, PS4Controller.Button.kLeftBumper.value).onTrue(new armExtendToValue(m_armsystem, 5000));


    // **************************************************
    // SHOULDER SYSTEM

    //Button To raise shoulder -- Uses A Button
    new JoystickButton(driverController, PS4Controller.Button.kCross.value).whileTrue(new lowerShoulder(m_shouldersystem, m_switchsystem));
    //Button To lower shoulder -- Uses Y Button
    new JoystickButton(driverController, PS4Controller.Button.kTriangle.value).whileTrue(new raiseShoulder(m_shouldersystem, m_switchsystem));
    //Reset Encoder
    new JoystickButton(driverController, PS4Controller.Button.kShare.value).whileTrue(new shoulderResetEncoder(m_shouldersystem));
    
    // **************************************************
    // Automatic droppers

    // Button to drop box on 2nd level
    //final TriggerR2Button BoxTwoBt = new TriggerR2Button(driverController);
    //new Trigger(BoxTwoBt::get).onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem, m_drivesystem, 2));
    new JoystickButton(driverController, PS4Controller.Button.kR2.value).onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem, m_drivesystem, 2));


    // Button to drop cone on 1st level
    //new JoystickButton(driverController, PS4Controller.Button.kRightBumper.value).onTrue(new DropCone(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem));
  
    // Button to drop box on 1st level
    new JoystickButton(driverController, PS4Controller.Button.kR1.value).onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem,m_drivesystem, 1));
 
    // Button to drop to floor level
    final POVButton DropFloorBt = new POVButton(driverController,Constants.POV_UP); 
    DropFloorBt.onTrue(new DropBoxTelOp(m_shouldersystem, m_armsystem, m_clampsystem,m_switchsystem, m_drivesystem, 0));

    // Button to pick up object
    final POVButton PickUpBt = new POVButton(driverController,Constants.POV_DOWN); 
    PickUpBt.onTrue(new PickUp(m_armsystem, m_clampsystem,m_switchsystem));

    final POVButton RaiseToShelf = new POVButton(driverController, Constants.POV_RIGHT);
    RaiseToShelf.onTrue(new RotateShoulderToValue(m_shouldersystem, 620));

    final POVButton PickUpOut = new POVButton(driverController, Constants.POV_LEFT);
    PickUpOut.onTrue(new PickUpOutside(m_armsystem, m_clampsystem, m_switchsystem, m_shouldersystem));

    //Button to toggle slow down
    new JoystickButton(driverController, PS4Controller.Button.kL1.value).onTrue(new toggleSlow_down(m_drivesystem));
  
    // Button to drop cone on 2nd level
    //final TriggerL2Button ConeTwoBt = new TriggerL2Button(driverController);
    //new Trigger(ConeTwoBt::get).onTrue(new AdjustCone(m_drivesystem, m_shouldersystem, m_limelightsystem));
    new JoystickButton(driverController, PS4Controller.Button.kL2.value).onTrue(new AdjustCone(m_drivesystem, m_shouldersystem, m_limelightsystem));

  
    

    // **************************************************
    // CLAMP SYSTEM

    // Button to clamp -- Uses Right Stick Button
    new JoystickButton(driverController, PS4Controller.Button.kOptions.value).onTrue(new ToggleClamp(m_clampsystem));



    /*
    *
    End of Main driver controller mapping; beginning of co-driver controller mappings 
    * 
    */

    //new JoystickButton(codriverController, PS4Controller.Button.kStart.value).whileTrue(new updateShoulderPID(m_shouldersystem));

    
    
  }
  private void configureLimitSwitches(){
      // new Trigger(m_switchsystem.switchArmIn::get).whileTrue(new armResetEncoder(m_armsystem)); // command to reset encoder, called when limit switch is pressed
      // new Trigger(m_switchsystem.switchShoulderIn::get).onTrue(new shoulderResetEncoder(m_shouldersystem));
      
    }
  public PS4Controller getDriverController() {
    return driverController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public Command getAutonomousCommand() {
    // return new DropCone(m_drivesystem, m_shouldersystem, m_armsystem, m_clampsystem, m_switchsystem, m_limelightsystem);
    m_shouldersystem.resetEncoder();
    return m_chooser.getSelected();
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
        SmartDashboard.putNumber("Compass Heading", m_drivesystem.getCompassHeading());
        SmartDashboard.putNumber("Fused Heading", m_drivesystem.getFusedHeading());
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
        SmartDashboard.putNumber("Shoulder Volts 1", m_shouldersystem.getShoulderMotorOneVoltage());
        SmartDashboard.putNumber("Shoulder Volts 2", m_shouldersystem.getShoulderMotorTwoVoltage());
        SmartDashboard.putNumber("Pitch", m_drivesystem.getPitch());
        SmartDashboard.putNumber("Roll", m_drivesystem.getRoll());
        SmartDashboard.putNumber("Angle", m_drivesystem.getAngle());
        SmartDashboard.putNumber("Yaw", m_drivesystem.getYaw());
        SmartDashboard.putBoolean("Clamp Open", m_clampsystem.isOpenClamp());
        SmartDashboard.putNumber("Slowdown Factor", m_drivesystem.getSlowdownFactor());
        SmartDashboard.putNumber("Distance to Wall", m_drivesystem.getDistanceToWall());
        SmartDashboard.putBoolean("Speed is 100", m_drivesystem.slowdownIs100());
        SmartDashboard.putBoolean("Speed is 80", m_drivesystem.slowdownIs80());
        SmartDashboard.putBoolean("Speed is 65", m_drivesystem.slowdownIs65());
        SmartDashboard.putBoolean("Near Wall", m_drivesystem.inRangeOfWall());

        //SmartDashboard.putString("Auto Mode", m_chooser.getSelected().getName());
        
        Constants.PID_SHOULDER_P = SmartDashboard.getNumber("Shoulder P", 0.04);
        Constants.PID_SHOULDER_I = SmartDashboard.getNumber("Shoulder I", 0);
        Constants.PID_SHOULDER_D = SmartDashboard.getNumber("Shoulder D", 0);
  }
}
