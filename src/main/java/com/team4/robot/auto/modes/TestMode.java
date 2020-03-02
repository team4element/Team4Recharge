package com.team4.robot.auto.modes;

import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.DriveOpenLoopAction;


// used to test auto modes before making them final
public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
        // runAction(new ResetPoseFromPathAction(new TestPath()));
        // runAction(new DrivePathAction(new TestPath()));

        runAction(new DriveOpenLoopAction(.18,-.18, .5));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90), 10, true));
    }
}
