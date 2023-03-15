package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
public class ClampSystem extends SubsystemBase{

    public PneumaticsControlModule pcm = new PneumaticsControlModule(Constants.PCM_CAN_ID);
    public Compressor compressor = new Compressor(5, PneumaticsModuleType.CTREPCM);
    //Look specifically at double solenoid type and methods
    public DoubleSolenoid clamp = new DoubleSolenoid(5, PneumaticsModuleType.CTREPCM, Constants.CLAMP_SOLENOID_PORTA, Constants.CLAMP_SOLENOID_PORTB);

    public ClampSystem() {
        clamp.set(Value.kForward);

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
        return !(clamp.get() == Value.kForward);
    }
}