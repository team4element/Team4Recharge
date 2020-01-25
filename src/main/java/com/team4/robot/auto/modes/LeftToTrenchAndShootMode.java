package com.team4.robot.auto.modes;

import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.TurnToHeadingAction;

import edu.wpi.first.wpilibj.Timer;

public class LeftToTrenchAndShootMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));
        runAction(new AutoSteerAndDistanceAction(245, 10));
        runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}