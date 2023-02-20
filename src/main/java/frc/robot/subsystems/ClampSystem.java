package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
public class ClampSystem extends SubsystemBase{

    public Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public DoubleSolenoid clamp = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLAMP_SOLENOID_PORTA, Constants.CLAMP_SOLENOID_PORTB);

    public ClampSystem() {

    }

    @Override
    public void periodic() {
        //Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        //Called once per scheduler run when in simulation
    }

    public void closeClamp() {
        clamp.set(Value.kForward);
    }

    public void openClamp() {
        clamp.set(Value.kReverse);
    }

    public boolean isOpenClamp() {
        return clamp.get() == Value.kForward;
    }
}