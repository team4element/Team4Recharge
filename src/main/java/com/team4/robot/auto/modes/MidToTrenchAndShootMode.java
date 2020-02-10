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
import com.team4.robot.actions.IntakeThroughPathAction;
import com.team4.robot.actions.ResetPoseFromPathAction;
import com.team4.robot.actions.WaitForPathMarkerAction;
import com.team4.robot.paths.trench.MiddleTrenchPath1;
import com.team4.robot.paths.trench.MiddleTrenchPath2;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.states.superstructure.SuperstructureState;

import edu.wpi.first.wpilibj.Timer;

public class MidToTrenchAndShootMode extends AutoModeBase{
    
    PathContainer path1;
    PathContainer path2;

    public MidToTrenchAndShootMode(){
        path1 = new MiddleTrenchPath1();
        path2 = new MiddleTrenchPath2();
    }

    
    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        // runAction(new ResetPoseAction(MidStartToTrenchPoints.startPose));
        runAction(new ResetPoseFromPathAction(path1));
        runAction(new AutoSteerAndDistanceAction(160, 1.5));
        runAction(new WaitAction(3));
        // runAction(new ShootAction(3));
        runAction(new ParallelAction(Arrays.asList(new DrivePathAction(path1), 
                    new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("Start Intake"),
                    new IntakeThroughPathAction(false))))));
        runAction(new WaitAction(.25));
        Superstructure.getInstance().setControlState(SuperstructureState.IDLE);
        runAction(new DrivePathAction(path2));
        runAction(new WaitAction(.25));
        runAction(new AutoSteerAndDistanceAction(200, 1.5)); // 245 for comp
        runAction(new WaitAction(3));
        // runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}