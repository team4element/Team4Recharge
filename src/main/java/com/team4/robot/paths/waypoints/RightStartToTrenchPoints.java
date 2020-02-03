package com.team4.robot.paths.waypoints;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class RightStartToTrenchPoints {
    //Divided into two separate trajectories

    //Trajectory 1:
    public static final Pose2d startPose1 = new Pose2d(new Translation2d(105, 145), Rotation2d.fromDegrees(180));
    public static final Pose2d finishIntakePhase1 = new Pose2d(new Translation2d(220, 145), Rotation2d.fromDegrees(180));
    public static final Pose2d finishPose1 = new Pose2d(new Translation2d(270, 135), Rotation2d.fromDegrees(180));

    //Trajectory 2:
    public static final Pose2d startPose2 = new Pose2d(new Translation2d(270, 135), Rotation2d.fromDegrees(0));
    public static final Pose2d allignShootPose = new Pose2d(new Translation2d(220, 145), Rotation2d.fromDegrees(0));
    public static final Pose2d finishPose2 = new Pose2d(new Translation2d(140, 145), Rotation2d.fromDegrees(0-15));
}