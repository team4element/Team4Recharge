package com.team4.robot.paths.waypoints;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class MidStartToTrenchPoints{
    public static final Pose2d startPose = new Pose2d(new Translation2d(105, 75), Rotation2d.fromDegrees(0));
    public static final Pose2d allignIntakePose = new Pose2d(new Translation2d(205, 120), Rotation2d.fromDegrees(180-45));
    public static final Pose2d startIntakePose = new Pose2d(new Translation2d(230, 155), Rotation2d.fromDegrees(180));
    public static final Pose2d finishIntakePose = new Pose2d(new Translation2d(270, 155), Rotation2d.fromDegrees(180));

    public static final Pose2d startPose2 = new Pose2d(new Translation2d(250, 145), Rotation2d.fromDegrees(0));
    public static final Pose2d allignShootPose = new Pose2d(new Translation2d(160, 90), Rotation2d.fromDegrees(0));
    public static final Pose2d finalPose = new Pose2d(new Translation2d(160, 145), Rotation2d.fromDegrees(0));
}