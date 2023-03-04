package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
public class ArmSystem extends SubsystemBase{

    
    private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR);
    private Encoder arm_encoder = new Encoder(Constants.ENCODER_ARM_A, Constants.ENCODER_ARM_B, false, Encoder.EncodingType.k4X);
    public double destination;
    private switchesSystem m_switchessystem;


    public ArmSystem(switchesSystem sub1) {
        m_switchessystem = sub1;
    }


    public void stopArmMotor() {
        armMotor.stopMotor();
    }

    @Override
    public void periodic() {
        //Called once per scheduler run
        if(m_switchessystem.isArmIn()){
            resetPosition();
        }
    }

    @Override
    public void simulationPeriodic() {
        //Called once per scheduler run when in simulation
    }

    //Following methods are for controlling the system
    public void updateDestination(double dest){
        destination = dest;
    }

    public void extendArm(double speed) {
        //System.out.println(speed);

        if (speed > Constants.MAX_ARM_VELOCITY) { 
            speed = Constants.MAX_ARM_VELOCITY;
        }
        if (speed < -Constants.MAX_ARM_VELOCITY) { 
            speed = -Constants.MAX_ARM_VELOCITY;
        }
        if (getPosition() > Constants.MAX_ARM_POSITION && speed > 0) {
            speed = 0;
        }
        /*if (getPosition()==0 &&  == 0) {
            speed = -.04;
        }*/

        if (m_switchessystem.isArmIn() == true && speed < 0) {
            speed = 0;
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
}