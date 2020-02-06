package com.team4.robot.auto.modes;

import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.lib.path.PathContainer;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.DrivePathAction;
import com.team4.robot.actions.ResetPoseFromPathAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.paths.trench.RightTrenchPath1;
import com.team4.robot.paths.trench.RightTrenchPath2;

import edu.wpi.first.wpilibj.Timer;

public class RightToTrenchAndShootMode extends AutoModeBase{
    
    PathContainer path1;
    PathContainer path2;

    public RightToTrenchAndShootMode(){
        path1 = new RightTrenchPath1();
        path2 = new RightTrenchPath2();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        runAction(new ResetPoseFromPathAction(path1));
        runAction(new AutoSteerAndDistanceAction(180, 1.5));
        runAction(new ShootAction(2.5));
        runAction(new DrivePathAction(path1));
        runAction(new WaitAction(1));
        runAction(new DrivePathAction(path2));
        runAction(new WaitAction(.5));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));
        runAction(new AutoSteerAndDistanceAction(230, 10));
        runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}