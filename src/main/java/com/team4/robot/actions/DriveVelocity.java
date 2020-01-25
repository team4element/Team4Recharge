package com.team4.robot.actions;

import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class DriveVelocity implements Action{
    
    private final Drive mDrive = Drive.getInstance();

    private double mSpeed, mDuration, mStartTime;
/**
 * 
 * @param speed velocity value, should be in feet per second
 * @param duration time of action, should be in seconds
 */
    public DriveVelocity(double speed, double duration){
        mSpeed = speed * 12;
        mDuration = duration;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();

    }

    @Override
    public void update() {
        mDrive.setVelocityInchesPerSecond(new DriveSignal(mSpeed, mSpeed));
    }   

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mDuration;
    }

    @Override
    public void stop() {
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }

}