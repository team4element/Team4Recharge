package com.team4.robot.auto.actions;

import com.team254.lib.geometry.Translation2d;
import com.team4.lib.actionbase.Action;
import com.team4.robot.RobotState;

public class WaitUntilInsideRegion implements Action {
    private final static RobotState mRobotState = RobotState.getInstance();

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight) {
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
    }

    @Override
    public boolean isFinished() {
        Translation2d position = mRobotState.getLatestFieldToVehicle().getValue().getTranslation();
        return position.x() > mBottomLeft.x() && position.x() < mTopRight.x()
                && position.y() > mBottomLeft.y() && position.y() < mTopRight.y();
    }

    @Override
    public void update() {}

    @Override
    public void stop() {}

    @Override
    public void start() {}
}
