package com.team4.robot.auto.modes;

import java.util.Arrays;

import com.team254.lib.geometry.Rotation2d;
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
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.TurnToHeadingAction;
import com.team4.robot.actions.WaitForPathMarkerAction;
import com.team4.robot.paths.rendezvous.MiddleRendPath1;
import com.team4.robot.paths.rendezvous.MiddleRendPath2;

import edu.wpi.first.wpilibj.Timer;

public class MidToRendAndShootMode extends AutoModeBase{
    
    PathContainer path1;
    PathContainer path2;

    public MidToRendAndShootMode(){
        path1 = new MiddleRendPath1();
        path2 = new MiddleRendPath2();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        
        final double startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseFromPathAction(path1));
        runAction(new AutoSteerAndDistanceAction(170, 1));
        // runAction(new ShootAction(2));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
        runAction(new ParallelAction(Arrays.asList(new DrivePathAction(path1), 
        new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("Begin Intake"),
        new IntakeThroughPathAction())))));

        runAction(new WaitAction(1));
        // runAction(new DrivePathAction(path2));
        // runAction(new WaitAction(.5));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(60), 1, true));
        runAction(new AutoSteerAndDistanceAction(230, 1.5)); //if follows path two make it 180
        runAction(new ShootAction(2));

        System.out.println(Timer.getFPGATimestamp() - startTime);

        // To prevent any unknown movement robot waits for 15 seconds after auto mode is complete during the auto period
        // and will not carry over to tele-op {@link Robot}
        runAction(new WaitAction(15));
    }
}