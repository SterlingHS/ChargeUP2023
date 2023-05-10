package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSystem;
import java.lang.Math;
/**
 *
 */
public class BalanceMiddlePlatform extends CommandBase {
    private DriveSystem m_drivesystem;
    private boolean inErrorRange;
    private boolean hasPitchChanged;
    private int stage = 0;
    private double startPitch;
    private double angle;
    private double signAngle;
    private double climbSpeed;
    private static final double firstClimbAngle = 17;
    private static final double climbingAngle =16;
    private static final double lowerClimbingBound = 14.5;
    private int directionClimb;
    public BalanceMiddlePlatform(DriveSystem sub1,int direction)
    
    {
        m_drivesystem = sub1;
        startPitch = Math.abs(m_drivesystem.getPitch());
        hasPitchChanged = true;
        climbSpeed = 0.4;
        directionClimb=direction;
        addRequirements(m_drivesystem);
    }
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            
            angle = Math.abs(m_drivesystem.getPitch())-startPitch;
            signAngle = m_drivesystem.getPitch()-startPitch;
            /* 
            if (!hasPitchChanged) {
                climbSpeed = 0.65*directionClimb;
                if (angle>firstClimbAngle) {
                    hasPitchChanged = true;
                }
            }
            else if (angle>climbingAngle) {//get up
                climbSpeed = 0.55*directionClimb;
            }
            else if (lowerClimbingBound<angle && angle<climbingAngle) {//climb
                climbSpeed = 0.34*directionClimb;
            }
            else {
                System.out.print("stoped");
                climbSpeed = 0;
                inErrorRange=true;

            }
            *\
            /*if (stage == 2) {
                climbSpeed = .4;
            }

            if (stage == 1) {
                if (angle < 20) {//Number TBD
                    stage = 2;
                }
                climbSpeed = .6;
            }

            if (stage == 0) {
                if (angle > 25) {//Number TBD
                    stage = 1;
                }
                climbSpeed = .6;
            }
            */

            // -35 < pitch < 35
            climbSpeed = signAngle*0.038;
            if(climbSpeed > .6) {
                climbSpeed = .6;
            }
            if(climbSpeed < -.6) {
                climbSpeed = -.6;
            }
            if (angle!=0){
            System.out.println("Angle"+signAngle);}
            m_drivesystem.forward(-climbSpeed);
            
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            m_drivesystem.stop();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
            // return (stage ==3  && inErrorRange);
        }

        @Override
        public boolean runsWhenDisabled() {
            return false;
        }
    
}

