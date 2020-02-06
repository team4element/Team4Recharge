package com.team4.robot.actions;

import com.team254.lib.control.Path;
import com.team254.lib.util.DriveSignal;
import com.team4.lib.actionbase.Action;
import com.team4.lib.path.PathContainer;
import com.team4.robot.subsystems.Drive;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private Drive mDrive = Drive.getInstance();
    private boolean mStopWhenDone;

    public DrivePathAction(PathContainer p, boolean stopWhenDone) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mStopWhenDone = stopWhenDone;
    }

    public DrivePathAction(PathContainer p) {
        this(p, true);
    }

    @Override
    public void start() {
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void stop() {
        if (mStopWhenDone) {
        
        mDrive.setOpenLoop(DriveSignal.BRAKE);    // mDrive.setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }

    }
}
