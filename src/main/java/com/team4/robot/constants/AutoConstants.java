package com.team4.robot.constants;

public class AutoConstants{

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveVelocityKp = .9; //0.9
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 10.0; //10.0
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps


    public static final double kDrivePositionKp = 1.5;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 2.0;
    public static final double kDrivePositionKf = 0.0;
    public static final int kDrivePositionIZone = 0;

    public static final int kDriveMaxVelocity = 18 * 12;
    public static final int kDriveMaxAccel = 20 * 12;

    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 12.0; // inches per second
    public static final double kMaxLookAhead = 48.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static final double kPathFollowingMaxAccel = 12.0 * 12.0;  // inches per second ^ 2
    public static final double kPathFollowingMaxVel = 16.0 * 12.0; // inches per second
    public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
    public static final double kPathFollowingProfileKi = 0.0;
    public static final double kPathFollowingProfileKv = 0.01 / 12.0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 0.003889;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0.001415;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = 0.1801 / 12.0;  // % throttle
    public static final double kPathFollowingGoalPosTolerance = 3.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;

    public static final double kShooterKp = 1.1;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 1.25;
    public static final double kShooterKf = .0725;
    public static final int kShooterIZone = 0;
    public static final double kShooterRampRate = 0.0;

    public static final double kLimelightAngleKp = .04;
    public static final double kLimelightAngleKi = .0;
    public static final double kLimelightAngleKd = 0.0;

    public static final double kLimelightDistanceKp = .02;
    public static final double kLimelightDistanceKi = .0;
    public static final double kLimelightDistancekd = .0;
}