package com.team4.robot.actions;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class TurnToHeadingAction implements Action {

    private Rotation2d mTargetHeading;
    private Drive mDrive = Drive.getInstance();

    private double mStartTime;
    private final double mDuration;
    private boolean mReset;

    public TurnToHeadingAction(Rotation2d heading) {
        mTargetHeading = heading;
        mDuration = 1.5;
        mReset = false;
    }

    public TurnToHeadingAction(Rotation2d heading, double duration, boolean reset) {
        mDuration = duration;
        mTargetHeading = heading;
        mReset = reset;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTurn() || (Timer.getFPGATimestamp() - mStartTime > mDuration);
    }

    @Override
    public void update() {
    }

    @Override
    public void stop() {
        System.out.println("Finished TurnToHeading!!!");
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
        mDrive.setBrakeMode(false);
    }

    @Override
    public void start() {
        if(mReset){
            mDrive.setHeading(Rotation2d.identity());
        }
        System.out.println("Starting TurnToHeading");
        mDrive.setWantTurnToHeading(mTargetHeading);
        mStartTime = Timer.getFPGATimestamp();
    }
}