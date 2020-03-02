package com.team4.robot.actions;

import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class DriveOpenLoopAction implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private double mStartTime;
    private final double mDuration, mLeft, mRight;

    private double startRot;

    public DriveOpenLoopAction(double left, double right, double duration) {
        mDuration = duration;
        mLeft = left;
        mRight = right;
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(mLeft, mRight));
        mStartTime = Timer.getFPGATimestamp();
        startRot = mDrive.getHeadingDouble();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration || Math.abs(mDrive.getHeadingDouble() - startRot) >= 45;
    }

    @Override
    public void stop() {
        mDrive.setBrakeMode(true);
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }
}