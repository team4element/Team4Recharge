package com.team4.robot.paths.waypoints;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class MidStartToRendPoints{
    public static final Pose2d startPose = new Pose2d(new Translation2d(105, 75), Rotation2d.fromDegrees(0));
    public static final Pose2d allignIntakePose = new Pose2d(new Translation2d(175, 25), Rotation2d.fromDegrees(180-30));
    public static final Pose2d beginIntakePose = new Pose2d(new Translation2d(225,20), Rotation2d.fromDegrees(180-25)); 
    public static final Pose2d finishIntakePose = new Pose2d(new Translation2d(240, 35), Rotation2d.fromDegrees(180+80));
    public static final Pose2d allignShootPose = new Pose2d(new Translation2d(190, -35), Rotation2d.fromDegrees(0));

    public static final Pose2d startPose2 = new Pose2d(new Translation2d(240, -15), Rotation2d.fromDegrees(0));
    public static final Pose2d finalPose = new Pose2d(new Translation2d(105, 75), Rotation2d.fromDegrees(0));


}