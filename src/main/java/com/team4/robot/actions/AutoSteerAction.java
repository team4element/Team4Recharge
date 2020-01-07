package com.team4.robot.actions;

import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.VisionTracker;

import edu.wpi.first.wpilibj.Timer;

public class AutoSteerAction implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private DelayedBoolean mDelayedEnd = null;
    private boolean mFinished = false;
    private boolean mReverse = false;
    private double mDuration, mStartTime;

    private CheesyDriveHelper mDriveHelper;

    public AutoSteerAction(boolean reverse, double duration) {
        mReverse = reverse;
        mDuration = duration;
        mDriveHelper = new CheesyDriveHelper();
    }


    @Override
    public void start() {
        mDelayedEnd = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        // Optional<AimingParameters> aimParams = Superstructure.getInstance().getLatestAimingParameters();
        double turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 0.1), -0.1);
        mDrive.setOpenLoop(mDriveHelper.cheesyDrive(0.3 * (mReverse ? -1.0 : 1.0), turn, true));
        mFinished = mDelayedEnd.update(Timer.getFPGATimestamp(), Timer.getFPGATimestamp() - mStartTime > mDuration);
    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }

    @Override
    public void stop() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
    }
}