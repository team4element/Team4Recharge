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
import com.team4.robot.paths.waypoints.RightStartToRendPoints;

import edu.wpi.first.wpilibj.Timer;

public class RightToRendAndShootMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
                
        double startTime =Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(RightStartToRendPoints.startPose));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToRendAndIntake.get(false), false));
        runAction(new TurnToHeadingAction(Rotation2d.fromRadians(0)));  
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightRendToStartAndShoot.get(false), false));
        runAction(new AutoSteerAndDistanceAction(160, 10));
        runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp()- startTime);
    }
}