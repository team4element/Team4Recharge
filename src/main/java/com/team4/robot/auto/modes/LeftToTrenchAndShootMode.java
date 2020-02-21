package com.team4.robot.auto.modes;

import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.lib.path.PathContainer;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.DrivePathAction;
import com.team4.robot.actions.DropIntakeAction;
import com.team4.robot.actions.ResetPoseFromPathAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.paths.trench.LeftTrenchPath1;
import com.team4.robot.paths.trench.LeftTrenchPath2;

import edu.wpi.first.wpilibj.Timer;

public class LeftToTrenchAndShootMode extends AutoModeBase{

    PathContainer path1;
    PathContainer path2;

    public LeftToTrenchAndShootMode(){
        path1 = new LeftTrenchPath1();
        path2 = new LeftTrenchPath2();
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        runAction(new ResetPoseFromPathAction(path1));
        runAction(new DropIntakeAction());
        runAction(new AutoSteerAndDistanceAction(180, 10));
        runAction(new ShootAction(3));
        runAction(new DrivePathAction(path1));
        runAction(new WaitAction(.25));
        runAction(new DrivePathAction(path2));
        runAction(new WaitAction(.25));
        runAction(new AutoSteerAndDistanceAction(245, 10));
        runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}