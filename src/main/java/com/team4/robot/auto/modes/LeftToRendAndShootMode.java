package com.team4.robot.auto.modes;

import java.util.Arrays;

import com.team4.lib.actionbase.ParallelAction;
import com.team4.lib.actionbase.SeriesAction;
import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.lib.path.PathContainer;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.DrivePathAction;
import com.team4.robot.actions.DropIntakeAction;
import com.team4.robot.actions.IntakeThroughPathAction;
import com.team4.robot.actions.ResetPoseFromPathAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.WaitForPathMarkerAction;
import com.team4.robot.paths.rendezvous.RightRendPath1;
import com.team4.robot.paths.rendezvous.RightRendPath2;

import edu.wpi.first.wpilibj.Timer;

public class LeftToRendAndShootMode extends AutoModeBase{

    PathContainer path1; 
    PathContainer path2;

    public LeftToRendAndShootMode(){
        path1 = new RightRendPath1();
        path2 = new RightRendPath2();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        runAction(new ResetPoseFromPathAction(path1));
        runAction(new AutoSteerAndDistanceAction(180, 1.5));
        runAction(new WaitAction(3));
        // runAction(new ShootAction(3));
        runAction(new ParallelAction(Arrays.asList(new DrivePathAction(path1),
                    new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("Begin Intake"), 
                    new IntakeThroughPathAction(false))))));
        runAction(new WaitAction(.25));
        runAction(new DrivePathAction(path2));
        runAction(new WaitAction(.25));
        runAction(new AutoSteerAndDistanceAction(180, 10));
        runAction(new WaitAction(3));
        // runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}