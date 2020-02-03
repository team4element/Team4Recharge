package com.team4.robot.constants;

public class DriveConstants {
    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveSlaveAId = 0;
    public static final int kLeftDriveSlaveBId = 2;
    public static final int kRightDriveMasterId = 4;
    public static final int kRightDriveSlaveAId = 3;
    public static final int kRightDriveSlaveBId = 5;
    
    public static final double kDriveEncoderPPR = 4096.0;

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 32;
    public static final double kDriveWheelDiameterInches = 6;
    public static final double kDriveWheelCircumferenceInches = Math.PI * kDriveWheelDiameterInches;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!    
    public static final double kDriveGearRatio = .633; //On actual bot 12d / 48d, 7.71/1  1/7.71, .129701

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.1044; //.135 // V per rad/s
    public static final double kDriveKa = 0.0463;  // V per rad/s^2
    public static final double kDriveKs = 0.894;

    // Geometry
    public static final double kCenterToFrontBumperDistance = 38.25 / 2.0;
    public static final double kCenterToRearBumperDistance = 38.25 / 2.0;
    public static final double kCenterToSideBumperDistance = 33.75 / 2.0;
    
    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches
}   