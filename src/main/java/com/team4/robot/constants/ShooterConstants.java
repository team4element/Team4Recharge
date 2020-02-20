package com.team4.robot.constants;

import com.team254.lib.util.Units;

public class ShooterConstants {
    public static final int kMasterMotorId = 5;
    public static final int kSlaveMotorId = 6;
    
    public static final double kShooterHeight = Units.inches_to_meters(25);
    public static final double kShooterAngle = Math.PI/4;

    public static final double kTargetInMeters = Units.inches_to_meters(TargetingConstants.kFloorToTarget);

    public static final double kGravityConstant = 9.81;
    public static final double kDragCoeff = 7.062675;

    public static final double kDrag = ((kDragCoeff * 1.117 * Math.PI * (Math.pow(.0889, 2)/2)));
    public static final double kMass = .15;

    public static final double kShooterFlyWheelRadius = Units.inches_to_meters(4);

    public static final double kShooterMaxSpeed = 130; //in meters per second
    public static final double kShooterTimeConstraint = 2.5; // in seconds

    public static final double kShooterGearRatio = 1.925; // 1 on comp bot
    public static final double kShooterEnconderPPR = 2048.0;

}