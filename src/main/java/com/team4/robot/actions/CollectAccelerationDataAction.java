package com.team4.robot.actions;

import java.util.List;

import com.team254.lib.physics.DriveCharacterization;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import com.team4.lib.actionbase.Action;
import com.team4.robot.constants.DriveConstants;
import com.team4.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class CollectAccelerationDataAction implements Action {
    private static final double kStartPower = 0.1;
    private static final double kPower = 0.8;
    private static final double kTotalTime = 2.0; // how long to run the test for
    private static final double kStartTime = 1.0;
    private static final Drive mDrive = Drive.getInstance();

    private final ReflectingCSVWriter<DriveCharacterization.DataPoint> mCSVWriter;
    private final List<DriveCharacterization.DataPoint> mAccelerationData;
    private final boolean mTurn;
    private final boolean mReverse;

    private double mStartTime = 0.0;
    private double mPrevVelocity = 0.0;
    private double mPrevTime = 0.0;

    /**
     * @param data     reference to the list where data points should be stored
     * @param highGear use high gear or low
     * @param reverse  if true drive in reverse, if false drive normally
     * @param turn     if true turn, if false drive straight
     */
    public CollectAccelerationDataAction(List<DriveCharacterization.DataPoint> data, boolean reverse, boolean turn) {
        mAccelerationData = data;
        mReverse = reverse;
        mTurn = turn;
        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/ACCEL_DATA.csv", DriveCharacterization.DataPoint.class);
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kStartPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kStartPower));
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
    }

    @Override
    public void update() {
        double currentVelocity;
        double currentTime;
        synchronized (mDrive) {
            currentVelocity = mDrive.getAverageDriveVelocityMagnitude() / DriveConstants.kDriveWheelRadiusInches; // rad/s
            currentTime = mDrive.getTimestamp();
        }

        // don't calculate acceleration until we've populated prevTime and prevVelocity
        if (mPrevTime == mStartTime) {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        if (currentTime - mStartTime > kStartTime) {
            mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * kPower));
        } else {
            return;
        }

        double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);

        // ignore accelerations that are too small
        if (acceleration < Util.kEpsilon) {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        mAccelerationData.add(new DriveCharacterization.DataPoint(
                currentVelocity, // convert to radians per second
                kPower * 12.0, // convert to volts
                currentTime - mStartTime
        ));

        mCSVWriter.add(mAccelerationData.get(mAccelerationData.size() - 1));

        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kTotalTime + kStartTime;
    }

    @Override
    public void stop() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mCSVWriter.flush();
    }
}