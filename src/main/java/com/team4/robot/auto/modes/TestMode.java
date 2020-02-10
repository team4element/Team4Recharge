package com.team4.robot.auto.modes;

import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.DrivePathAction;
import com.team4.robot.actions.ResetPoseFromPathAction;
import com.team4.robot.paths.rendezvous.MiddleRendPath1;
import com.team4.robot.subsystems.Drive;


// used to test auto modes before making them final
public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
        Drive.getInstance().startLogging();
    
        runAction(new ResetPoseFromPathAction(new MiddleRendPath1()));
        runAction(new DrivePathAction(new MiddleRendPath1()));

        Drive.getInstance().stopLogging();
    }
}
