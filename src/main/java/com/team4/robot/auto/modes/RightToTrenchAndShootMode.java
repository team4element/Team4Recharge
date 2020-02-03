package com.team4.robot.auto.modes;

import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.DriveTrajectory;
import com.team4.robot.actions.ResetPoseAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.TurnToHeadingAction;
import com.team4.robot.paths.TrajectoryGenerator;
import com.team4.robot.paths.waypoints.RightStartToTrenchPoints;

import edu.wpi.first.wpilibj.Timer;

public class RightToTrenchAndShootMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(RightStartToTrenchPoints.startPose1));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToTrenchAndIntake.get(false), false));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));
        runAction(new ResetPoseAction(RightStartToTrenchPoints.startPose2));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightTrenchToTrenchAndShoot.get(false), false));
        runAction(new AutoSteerAndDistanceAction(200, 10));
        runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}