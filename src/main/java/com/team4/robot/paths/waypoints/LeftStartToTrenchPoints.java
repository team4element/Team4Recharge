package com.team4.robot.paths.waypoints;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class LeftStartToTrenchPoints{
    //This is divided into two trajectories for ease on the robot

    //Trajectory 1
    public static final Pose2d startPose1 = new Pose2d(new Translation2d(105, 15), Rotation2d.fromDegrees(180));
    public static final Pose2d beginIntakePose = new Pose2d(new Translation2d(225, 150), Rotation2d.fromDegrees(180));
    public static final Pose2d finishIntakePose = new Pose2d(new Translation2d(345, 150), Rotation2d.fromDegrees(180));
    public static final Pose2d finalPose1 = new Pose2d(new Translation2d(395, 140), Rotation2d.fromDegrees(180));

    //Trajectory 2
    public static final Pose2d startPose2 = new Pose2d(new Translation2d(395, 140), Rotation2d.fromDegrees(0));
    public static final Pose2d allignPose = new Pose2d(new Translation2d(340, 150), Rotation2d.fromDegrees(0));
    public static final Pose2d finalPose2 = new Pose2d(new Translation2d(220, 140), Rotation2d.fromDegrees(0+15));
}