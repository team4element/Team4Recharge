package com.team4.robot.paths.waypoints;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class LeftStartToRendPoints{
    public static final Pose2d startPose = new Pose2d(new Translation2d(105, 15), Rotation2d.fromDegrees(180.0));
    public static final Pose2d beginIntakePose = new Pose2d(new Translation2d(215, 15), Rotation2d.fromDegrees(180.0));
    public static final Pose2d finishIntakePose = new Pose2d(new Translation2d(235, -5), Rotation2d.fromDegrees(180.0-20));
    public static final Pose2d finalPose = new Pose2d(new Translation2d(105, 15), Rotation2d.fromDegrees(180+20)); 
}