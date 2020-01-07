package com.team4.robot.actions;

import com.team4.lib.actionbase.RunOnceAction;
import com.team4.robot.subsystems.VisionTracker;
import com.team4.robot.subsystems.VisionTracker.LedMode;

public class LEDOffAction extends RunOnceAction{
    
    VisionTracker mVisionTracker;

    public LEDOffAction(){
        mVisionTracker = VisionTracker.getInstance();
    }

    
    @Override
    public void runOnce() {
        mVisionTracker.setLEDMode(LedMode.OFF);
    }
}