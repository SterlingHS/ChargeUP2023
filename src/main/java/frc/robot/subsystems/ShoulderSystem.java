package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class ShoulderSystem extends SubsystemBase { 

    private static Encoder shoulder_encoder;
    private WPI_TalonSRX shoulderMotor1;
    private WPI_TalonSRX shoulderMotor2;
    private MotorControllerGroup shoulderMotorGroup;
    private switchesSystem m_switchsystem;


    public ShoulderSystem(switchesSystem sub1) {
        shoulder_encoder = new Encoder(Constants.ENCODER_SHOULDER_A, Constants.ENCODER_SHOULDER_B);
        shoulderMotor1 = new WPI_TalonSRX(Constants.SHOULDER_MOTOR_ONE);
        shoulderMotor2 = new WPI_TalonSRX(Constants.SHOULDER_MOTOR_TWO);
        shoulderMotorGroup = new MotorControllerGroup(shoulderMotor1, shoulderMotor2);
        m_switchsystem = sub1;
    }

    @Override
    public void periodic() {
        //Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        //Called once per scheduler run when in simulation
    }

    
    public void raiseShoulder(double speed) {
        if (speed > Constants.MAX_SHOULDER_VELOCITY) {
            speed = Constants.MAX_SHOULDER_VELOCITY;
        }
        if (speed < -Constants.MAX_SHOULDER_VELOCITY) { 
            speed = -Constants.MAX_SHOULDER_VELOCITY;
        }
        if (m_switchsystem.isShoulderIn() == true && speed < 0) {
            speed = 0;
          }
        shoulderMotorGroup.set(speed);

    }

    public void stopShoulderMotor() {
        shoulderMotorGroup.stopMotor();
    }

    public int getPosition() {
        return shoulder_encoder.get();
    }
}
