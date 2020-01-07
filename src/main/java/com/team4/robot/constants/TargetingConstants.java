package com.team4.robot.constants;

public class TargetingConstants {
      // limelight
      public static final double kHorizontalFOV = 59.6; // degrees
      public static final double kVerticalFOV = 49.7; // degrees
      public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
      public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
      public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
  
      public static final double kFloorToLens = 7.25;
      public static final double kFloorToLensAngle = 15.09;
      public static final double kFloorToTarget = 0;

      public static final double kMaxTrackerDistance = 9.0;
      public static final double kMaxGoalTrackAge = 2.5;
      public static final double kMaxGoalTrackAgeNotTracking = 0.1;
      public static final double kMaxGoalTrackSmoothingTime = 0.5;
      public static final double kTrackStabilityWeight = 0.0;
      public static final double kTrackAgeWeight = 10.0;
      public static final double kTrackSwitchingWeight = 100.0;
  
      public static final double kCameraFrameRate = 90.0;
      public static final double kMinStability = 0.5;

      public static final double kLimelightKp = 1;
      public static final double kLimelightki = .0;
      public static final double kLimelightkd = 0;
}