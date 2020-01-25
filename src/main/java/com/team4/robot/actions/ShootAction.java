package com.team4.robot.actions;

import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.VisionTracker;
import com.team4.robot.subsystems.states.SuperstructureState;

import edu.wpi.first.wpilibj.Timer;

public class ShootAction implements Action{
    
    private double mDuration, mStartTime;

    private final Superstructure mSuperstructure = Superstructure.getInstance();

    public ShootAction(double duration){
        mDuration = duration;
    }
    
    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        VisionTracker.getInstance().setVisionEnabled(true);
        mSuperstructure.setControlState(SuperstructureState.Convey_Shoot);
    }
    
    @Override
    public void update() {
        
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mDuration;
    }

    @Override
    public void stop() {
        VisionTracker.getInstance().setVisionEnabled(false);
        Drive.getInstance().setBrakeMode(false);
        mSuperstructure.setControlState(SuperstructureState.IDLE);
    }
}