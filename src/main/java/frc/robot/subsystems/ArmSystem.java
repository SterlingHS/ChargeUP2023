package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
public class ArmSystem extends SubsystemBase{
    private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR);
    private Encoder arm_encoder = new Encoder(Constants.ENCODER_ARM_A, Constants.ENCODER_ARM_B, false, Encoder.EncodingType.k4X);
public void stopArmMotor(){
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
    if (speed > 0.4)
    { 
      speed = 0.4;
    }
    if (speed < -0.4)
    { 
      speed = -0.4;
    }
      armMotor.set(speed); 
}

public double getPosition() {
    return arm_encoder.get();
}}