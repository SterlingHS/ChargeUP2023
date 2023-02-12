package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class ArmSystem {
    private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.ARM_MOTOR);

public void stop(){
    armMotor.stopMotor();
}}