package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.util.Units;

//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.wpilibj.Encoder;
/**
 *
 */


//Actually DriveSytemPaul
public class DriveSystem extends SubsystemBase {
    private WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT);
    private WPI_TalonSRX leftRear = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_BACK);
    private WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT);
    private WPI_TalonSRX rightRear = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_BACK);
    public MotorControllerGroup mLeft = new MotorControllerGroup(leftFront, leftRear);
    public MotorControllerGroup mRight = new MotorControllerGroup(rightFront, rightRear);
    private DifferentialDrive mDrive = new DifferentialDrive(mLeft, mRight);
    public double DRIVER_SLOWDOWN;
    private static double lastSpeed;
    private static double lastAcceleration;
    private static double deltaSpeed;
    private static double deltaAcceleration;//deltaSpeed and delta Acceleration are also target acceleration and target jerk;
    private static double targetAcceleration;
    //private static double targetJerk; commented out
    // public static final DifferentailDriveKinematics KDriveKinematics = new
    // DifferentailDriveKinematics(kTrackWidthMeters)

    private AHRS navx_device = new AHRS(SerialPort.Port.kMXP);

    public DriveSystem() {
        leftRear.setInverted(true);
        leftFront.setInverted(true);
        rightRear.setInverted(false);
        rightFront.setInverted(false);
        leftFront.setSensorPhase(false);
        lastSpeed = 0;
        lastAcceleration = 0;
        DRIVER_SLOWDOWN = 0.8;

        // inverted talon sensor

        // 2208 pulses per 10ft
        //navx_device.enableLogging(true);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void arcDrive(double xSpeed, double zRotation, double slowdown_factor) {
        if (true)//this is where IsJoystick will go
        {
            xSpeed = -xSpeed;
            zRotation = -zRotation;
        }



        if (slowdown_factor < 1 && slowdown_factor >= 0) {
            xSpeed *= DRIVER_SLOWDOWN;
            zRotation *= DRIVER_SLOWDOWN;
        }
        xSpeed = controlSpeed(xSpeed);
        mDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void arcademDrive(double xSpeed, double zRotation) {
        System.out.println(xSpeed);
        mDrive.arcadeDrive(-xSpeed, -zRotation);
    }


    private double controlSpeed(double targetSpeed) {
            deltaSpeed = targetSpeed-lastSpeed;
            deltaAcceleration = deltaSpeed-lastAcceleration;//deltaSpeed and delta Acceleration are also target acceleration and target jerk;
            targetAcceleration = deltaSpeed;
            // targetJerk = deltaAcceleration; //Commented out
            if (targetAcceleration != 0.0){
                /* 
                if (targetJerk > Constants.MAX_JERK)
                {
                    targetAcceleration = Constants.MAX_JERK+lastAcceleration;
                }
                else if (targetJerk < -Constants.MAX_JERK)
                {
                    targetAcceleration = -Constants.MAX_JERK+lastAcceleration;
                }
                */
                if (targetAcceleration > Constants.MAX_ACCELERATION) 
                {
                    
                    deltaSpeed = lastSpeed+Constants.MAX_ACCELERATION; // if the target Acceleration is positive add it to the Last speed
                }
                else if (targetAcceleration < Constants.MAX_ACCELERATION && targetAcceleration >0)
                {
                    deltaSpeed = lastSpeed+deltaAcceleration; //if the target Acceleration is positive but smaller than max add it to last speed
                }


                if(targetAcceleration < -Constants.MAX_ACCELERATION)
                {
                    deltaSpeed = lastSpeed-Constants.MAX_ACCELERATION; 
                }
                else if (targetAcceleration > -Constants.MAX_ACCELERATION && targetAcceleration < 0)
                {
                    deltaSpeed = lastSpeed+deltaAcceleration;
                }
                //this if else handles negative target accelerations the sign difference is because deltaAcceleration is negative in this look, but MAX_ACCELERATION is never negative.
        }
        else {
            deltaSpeed = targetSpeed;
        }
        lastSpeed = deltaSpeed;
        if (deltaSpeed>DRIVER_SLOWDOWN) {
            deltaSpeed=DRIVER_SLOWDOWN;
        }
        return deltaSpeed;
    }
    
    public void toggleSlowdown() {
        if (DRIVER_SLOWDOWN == 1) {
            DRIVER_SLOWDOWN = 0.80;
        }
        else if ((DRIVER_SLOWDOWN == .8)) {
            DRIVER_SLOWDOWN = 0.65;
        }
        else{
            DRIVER_SLOWDOWN = 1;
        }
    }

    public void stop() {
        leftFront.stopMotor();
        rightFront.stopMotor();
        leftRear.stopMotor();
        rightRear.stopMotor();
    }

    public void turn(double speed) {
        arcademDrive(0, speed);
    }

    public void turnRight() {
        arcademDrive(0, 0.40);
    }

    public void turnLeft() {
        arcademDrive(0, -0.40);
    }

    public void turnSpeed(double speed) {
        arcademDrive(0, speed);
    }

    public void forward(double xspeed) {
        arcademDrive(-xspeed, 0);
    }

    public double read_distance_right_encoder() {
        return rightFront.getSelectedSensorPosition();
        // changed to work with talon encoders
    }

    public double read_distance_left_encoder() {
        return leftFront.getSelectedSensorPosition();
        // changed to work with talon encoders
    }

    public double read_distance_encoder() {
        return (rightFront.getSelectedSensorPosition() + leftFront.getSelectedSensorPosition()) / 2;
        // changed to work with talon encoders
    }

    public double read_velocity_encoder() {
        return (rightFront.getSelectedSensorVelocity() + leftFront.getSelectedSensorVelocity()) / 2;
        // changed to work with talon encoders
    }

    public void resetEncoders() {
        rightFront.setSelectedSensorPosition(0);
        leftFront.setSelectedSensorPosition(0);
    }

    public void calibrateGyro() {
        navx_device.calibrate();
    }

    public boolean iscalibrating() {
        return navx_device.isCalibrating();
    }

    public void resetAngle() {
        navx_device.reset();
    }

    public double getRightEncoder() {
        return rightFront.getSelectedSensorPosition();
    }
    public double getLeftEncoder() {
        return leftFront.getSelectedSensorPosition();
    }
    public double getRightDistance() {
        return encoderToDistanceMeters(rightFront.getSelectedSensorPosition());
    }
    public double getLeftDistance() {
        return encoderToDistanceMeters(leftFront.getSelectedSensorPosition());
    }
    public double getDistance() {
        return (getRightDistance() + getLeftDistance()) / 2;
    }

    public double getAngle() {
        return navx_device.getAngle();
    }

    public double getAngle360() {
        double angle = navx_device.getAngle();

        double correctedAngle = angle % 360;
        if (correctedAngle < 0) {
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

    public double getYaw() {
        return navx_device.getYaw();
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


    private double encoderToDistanceMeters(double sensorCounts){
        double kGearRatio = 8.45;
        double kWheelRadiusInches = 3;
        double kCountsPerRev = 4096;
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / kGearRatio;
        double positionMeters = 10 * wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        return positionMeters;
      }
    
      private int velocityToNativeUnits(double velocityMetersPerSecond){
        double kGearRatio = 8.45;
        double kWheelRadiusInches = 3;
        double kCountsPerRev = 4096;
        double k100msPerSecond = 10;
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
      }

      

}
// This adds the control speed function. This also has the capability to set a max jerk aswell. This only cares if acceleration is positive or negative and adds it to the value. I ran tests with 4 sets of numbers, each on being a different case and the program should output the correct value. MAX_ACCEL needs to be made in constants 