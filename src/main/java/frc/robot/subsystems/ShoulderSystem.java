package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class ShoulderSystem extends SubsystemBase { 

    private static Encoder shoulder_encoder;
    private WPI_TalonSRX shoulderMotor;


    public ShoulderSystem() {
        shoulder_encoder = new Encoder(Constants.ENCODER_SHOULDER_A, Constants.ENCODER_SHOULDER_B);
        shoulderMotor = new WPI_TalonSRX(Constants.SHOULDER_MOTOR);
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
        shoulderMotor.set(speed);
    }

    public void stopShoulderMotor() {
        shoulderMotor.stopMotor();
    }

    public int getPosition() {
        return shoulder_encoder.get();
    }
}
