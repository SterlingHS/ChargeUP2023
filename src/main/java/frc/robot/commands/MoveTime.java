package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;


/**
 *
 */
    public class MoveTime extends CommandBase {

    private final DriveSystem m_drivesystem;
    private long starting_time;

    private double time;
    private double speed;
    private double starting_distance;

 
    public MoveTime(DriveSystem sub, double speed1, double time1) {
        m_drivesystem = sub;
        time = time1*1000;
        speed = speed1;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        starting_distance = m_drivesystem.getDistance();
        start_timer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Speed: =====" + speed +"----"+m_drivesystem.getDistance());

        m_drivesystem.forward(speed);
        //m_drivesystem.mLeft.set(0.5);
        //m_drivesystem.mRight.set(0.5);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        double delta_d = m_drivesystem.getDistance() - starting_distance;
        System.out.println("***********\n"+"Distance Moved: "+delta_d+"\n*****************");
        m_drivesystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
     if(get_timer()>time) return true;

        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private void start_timer(){
        starting_time = System.currentTimeMillis();
    
      }
    
      private double get_timer(){
        double timer = System.currentTimeMillis() - starting_time;
        return timer;
      }
}