package com.team4.robot.auto.modes;

import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.DriveTrajectory;
import com.team4.robot.actions.ResetPoseAction;
import com.team4.robot.paths.TrajectoryGenerator;
import com.team4.robot.paths.waypoints.MidStartToTrenchPoints;
import com.team4.robot.subsystems.Drive;


// used to test auto modes before making them final
public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
        Drive.getInstance().startLogging();
        // double startTime =Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(MidStartToTrenchPoints.startPose));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToRendAndShoot.get(false), false));
        // runAction(new AutoSteerAndDistanceAction(156, 10));
        // runAction(new WaitAction(1));
        // runAction(new ShootAction(5));
        
        // runAction(new WaitAction(5));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90)));
        // runAction(new AutoSteerAndDistanceAction(15, 5));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(270.0)));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().retPath.get(true), true));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180.0)));
        // System.out.println(Timer.getFPGATimestamp()- startTime);
        Drive.getInstance().stopLogging();
    }
}
