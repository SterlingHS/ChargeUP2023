
package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;

//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.wpilibj.Encoder;
/**
 *
 */
public class DriveSystemTest extends SubsystemBase {

    private WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT);
    private WPI_TalonSRX leftRear  = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_BACK);
    private WPI_TalonSRX rightFront  = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT);
    private WPI_TalonSRX rightRear  = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_BACK);

    private MotorControllerGroup mLeft = new MotorControllerGroup(leftFront, leftRear);
    private MotorControllerGroup mRight = new MotorControllerGroup(rightFront, rightRear);
    private DifferentialDrive mDrive = new DifferentialDrive(mLeft, mRight);

    //public static final DifferentailDriveKinematics KDriveKinematics = new DifferentailDriveKinematics(kTrackWidthMeters)

    private AHRS navx_device = new AHRS(SerialPort.Port.kMXP);

    private PIDController m_forwardPIDController = new PIDController(1, 0, 0);
    private PIDController m_turnPIDController = new PIDController(1, 0, 0);
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(.1, 1),
                                                    new TrapezoidProfile.State(5, 0),
                                                    new TrapezoidProfile.State(0, 0));


   
    public DriveSystemTest() 
    {
        leftRear.setInverted(false);
        leftFront.setInverted(false);
        rightRear.setInverted(false);
        rightFront.setInverted(false);
        leftFront.setSensorPhase(true);
        
        navx_device.enableLogging(true);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void arcDrive(double xSpeed, double zRotation, double slowdown_factor) 
    {
        if(slowdown_factor < 1 && slowdown_factor >= 0)
        {
            xSpeed*=slowdown_factor;
            zRotation*=slowdown_factor;
        }

        mDrive.arcadeDrive(-xSpeed, -zRotation);
    }

    public void stop() {
        leftFront.stopMotor();
        rightFront.stopMotor();
        leftRear.stopMotor();
        rightRear.stopMotor();
    }

    public void turnRight() {
        arcDrive(0,0.5,1);
    }

    public void turnLeft(){
        arcDrive(0,-0.5,1);
    }

    public void forward(){
        arcDrive(0.5,0,1);
    }

    public void forwardSpeed(double xSpeed){
        arcDrive(xSpeed,0,1);
    }

    public void backward(){
        arcDrive(-0.5,0,1);
    }

    public double read_distance_right_encoder()
    {
        return rightFront.getSelectedSensorPosition();
        //changed to work with talon encoders
    }

    public double read_distance_left_encoder()
    {
        return leftFront.getSelectedSensorPosition();
        //changed to work with talon encoders
    }

    public double read_pulse_right_encoder()
    {
        return 0;//this does nothing
    }

    public double read_pulse_left_encoder()
    {
        return 0;//this does nothing
    }

    public double read_velocity_encoder() { 
        return (rightFront.getSelectedSensorVelocity()+leftFront.getSelectedSensorVelocity())/2;
        //changed to work with talon encoders
    }

    public void calibrateGyro()
    {
        navx_device.calibrate();
    }

    public boolean iscalibrating()
    {
        return navx_device.isCalibrating();
    }

    public void resetAngle()
    {
        navx_device.reset();
    }

    public double getAngle()
    {
        return navx_device.getAngle();
    }

    public double getAngle360()
    {
        double angle = navx_device.getAngle();

        double correctedAngle = angle % 360;
        if (correctedAngle < 0){
            correctedAngle += 360;
        }
        return correctedAngle;
    }
    
    public double getPitch() {
        return navx_device.getPitch();
    }


    public double getRoll() {
        return navx_device.getRoll();
    }

    public double getCompassHeading() {
        return navx_device.getCompassHeading();
    }

    public double getFusedHeading() {
        return navx_device.getFusedHeading();
    }

    public double getLinearWorldAccelX() {
        return navx_device.getWorldLinearAccelX();
    }

    public double getLinearWorldAccelY() {
        return navx_device.getWorldLinearAccelY();
    }

    public double getLinearWorldAccelZ() {
        return navx_device.getWorldLinearAccelZ();
    }
    
}




