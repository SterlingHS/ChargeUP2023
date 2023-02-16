package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
public class ArmSystem extends SubsystemBase{
    private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR);
    private Encoder arm_encoder = new Encoder(Constants.ENCODER_ARM_A, Constants.ENCODER_ARM_B, false, Encoder.EncodingType.k4X);
    private DigitalInput switchArmIn = new DigitalInput(Constants.DIO_SWITCH_ARM_IN);
    
    public void stopArmMotor() {
        armMotor.stopMotor();
    }

    @Override
    public void periodic() {
        //Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        //Called once per scheduler run when in simulation
    }

    //Following methods are for controlling the system

    public void extendArm(double speed) {
        System.out.println(speed);

        if (speed > Constants.MAX_ARM_VELOCITY) { 
            speed = Constants.MAX_ARM_VELOCITY;
        }
        if (speed < -Constants.MAX_ARM_VELOCITY) { 
            speed = -Constants.MAX_ARM_VELOCITY;
        }
        if (isArmIn() == true && speed < 0) {
            speed = 0;
            resetPosition();
        }
        armMotor.set(speed); 
    }

    // Reads position of arm encoder
    public double getPosition() {
        return arm_encoder.get();
    }

    // Resets position of arm encoder
    public void resetPosition() {
        arm_encoder.reset();
    }

    // Retuens true if arm is in, false if it is out
    public boolean isArmIn() {
        return !switchArmIn.get();
    }
}