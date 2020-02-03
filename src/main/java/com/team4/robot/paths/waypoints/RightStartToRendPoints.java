package com.team4.robot.paths.waypoints;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class RightStartToRendPoints {
    public static final Pose2d startPose = new Pose2d(new Translation2d(105,150), Rotation2d.fromDegrees(180));
    public static final Pose2d startIntake1stPose = new Pose2d(new Translation2d(250, 55), Rotation2d.fromDegrees(180-70));
    public static final Pose2d startIntake2ndPose = new Pose2d(new Translation2d(225, 20), Rotation2d.fromDegrees(180-90));
    public static final Pose2d finishIntakePose = new Pose2d(new Translation2d(230, -25), Rotation2d.fromDegrees(0));
    public static final Pose2d finalPose = new Pose2d(new Translation2d(105, 60), Rotation2d.fromDegrees(0+5));
}