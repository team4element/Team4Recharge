package com.team4.robot.auto.modes;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.DriveTrajectory;
import com.team4.robot.actions.ResetPoseAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.TurnToHeadingAction;
import com.team4.robot.paths.TrajectoryGenerator;
import com.team4.robot.paths.waypoints.MidStartToRendPoints;
import com.team4.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class MidToRendAndShootMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        
        double startTime = Timer.getFPGATimestamp();

        runAction(new ResetPoseAction(MidStartToRendPoints.startPose));
        runAction(new AutoSteerAndDistanceAction(160, 1));
        runAction(new ShootAction(2));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().midStartToRendAndIntake.get(true), false));
        runAction(new ResetPoseAction(new Pose2d(MidStartToRendPoints.startPose2.getTranslation(), Drive.getInstance().getHeading())));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(360), 1.2));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().midRendToStartAndShoot.get(false), false));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0), .5));
        runAction(new AutoSteerAndDistanceAction(156, 1.5));
        runAction(new ShootAction(2));

        System.out.println(Timer.getFPGATimestamp() - startTime);

        // To prevent any unknown movement robot waits for 15 seconds after auto mode is complete during the auto period
        // and will not carry over to tele-op {@link Robot}
        runAction(new WaitAction(15));
    }
}