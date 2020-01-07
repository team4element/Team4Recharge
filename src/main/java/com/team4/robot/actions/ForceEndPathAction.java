package com.team4.robot.actions;

import com.team4.lib.actionbase.RunOnceAction;
import com.team4.robot.subsystems.Drive;

public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        Drive.getInstance().forceDoneWithPath();
    }
}