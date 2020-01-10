package com.team4.robot.actions;

import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.lib.util.Subsystem;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;

public class SubsystemVelocityAction implements Action{
    
    private Subsystem mSubsystem;

    private double mSetpoint;
    private double mDuration, mStartTime;
    private boolean isFinished;

    /**
     * Runsdrive
     * 
     * @param setpoint is in inches per second
     */
    public SubsystemVelocityAction(Subsystem subsystem, double setpoint){
        mSetpoint = setpoint;
        mSubsystem = subsystem;
    }

    public SubsystemVelocityAction(Subsystem subsystem, double setpoint, double duration){
        this(subsystem, setpoint);
        mDuration = duration;
    }

    @Override
    public void start() {
        if(mSubsystem == Drive.getInstance()){
            if(mSetpoint > Drive.getInstance().getLinearVelocity()){
                Drive.getInstance().setVelocityInchesPerSecond(new DriveSignal(mSetpoint, mSetpoint));
            }
        }else if (mSubsystem == Shooter.getInstance()){
            if(mSetpoint > Shooter.getInstance().getVelocity()){
                Shooter.getInstance().setVelocity(mSetpoint, 0);
            }
        }else{
            //TODO: add logic for subsystems
        }
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        if(mSubsystem == Drive.getInstance()){
            isFinished = mSetpoint <= Drive.getInstance().getLinearVelocity();
        }else if(mSubsystem == Shooter.getInstance()){
            isFinished = mSetpoint <= Shooter.getInstance().getVelocity();
        }else{
            //TODO: add logic for subsystems
        }
    }

    @Override
    public boolean isFinished() {
        if(mDuration != Double.NaN){
            return Timer.getFPGATimestamp() - mStartTime <= mDuration;
        }else{
            return isFinished;
        }
    }

    @Override
    public void stop() {
        mSubsystem.stop();
    }
}