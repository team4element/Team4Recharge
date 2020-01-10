package com.team4.robot.actions;

import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;

public class DrivePositionAction implements Action{
    
    private final Drive mDrive = Drive.getInstance();
    private double mSetpoint;

    private double mInitPosition;
    private boolean isFinished;

    public DrivePositionAction(double setpoint){
        mSetpoint = setpoint;
    }

    @Override
    public void start() {
        mInitPosition = mDrive.getRightEncoderDistance();
        mDrive.setPosition(new DriveSignal(mSetpoint, mSetpoint));
    }

    @Override
    public void update() {
        isFinished = (mInitPosition + mSetpoint) <= mDrive.getRightEncoderDistance();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void stop() {
        mDrive.setOpenLoop(new DriveSignal(0, 0));    
    }


}