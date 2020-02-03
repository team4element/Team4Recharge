package com.team4.robot.auto.modes;

import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.DriveVelocity;
import com.team4.robot.subsystems.Drive;


// used to test auto modes before making them final
public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
        // Drive.getInstance().startLogging();
        // double startTime =Timer.getFPGATimestamp();
        // runAction(new ResetPoseAction(RightStartToTrenchPoints.startPose1));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightStartToTrenchAndIntake.get(false), false));
        // Drive.getInstance().resetEncoders();
        runAction(new DriveVelocity(-1, 3));
        // RobotState.getInstance().resetDistanceDriven();
        // runAction(new ResetPoseAction(Pose2d.identity()));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(0)));
        // runAction(new ResetPoseAction(RightStartToTrenchPoints.startPose2));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightTrenchToTrenchAndShoot.get(false), false));
        // runAction(new AutoSteerAndDistanceAction(156, 10));
        // runAction(new ShootAction(5));
        
        Drive.getInstance().stopLogging();
    }
}
