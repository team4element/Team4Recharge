package com.team4.robot.constants;

public class AutoConstants{

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveVelocityKp = 0.001; //0.9
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 10.0; //10.0
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps


    public static final double kDrivePositionKp = 0.053;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 2.0;
    public static final double kDrivePositionKf = 0.0;
    public static final int kDrivePositionIZone = 0;

    public static final int kDriveMaxVelocity = 16 * 12;
    public static final int kDriveMaxAccel = 16 * 12;

    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 12.0; // inches per second
    public static final double kMaxLookAhead = 48.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kShooterKp = 1.1;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 1.25;
    public static final double kShooterKf = .0725;
    public static final int kShooterIZone = 0;
    public static final double kShooterRampRate = 0.0;

    public static final double kLimelightAngleKp = .05;
    public static final double kLimelightAngleKi = .0;
    public static final double kLimelightAngleKd = .0;

    public static final double kLimelightDistanceKp = .02;
    public static final double kLimelightDistanceKi = .0;
    public static final double kLimelightDistancekd = .0;
}