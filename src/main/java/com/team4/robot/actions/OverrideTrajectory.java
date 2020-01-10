package com.team4.robot.actions;

import com.team4.lib.actionbase.RunOnceAction;
import com.team4.robot.subsystems.Drive;

public class OverrideTrajectory extends RunOnceAction {
    @Override
    public void runOnce() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
