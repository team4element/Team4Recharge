package com.team4.robot.auto.modes;

import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.subsystems.Drive;


// used to test auto modes before making them final
public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
        Drive.getInstance().startLogging();
        
        Drive.getInstance().stopLogging();
    }
}
