package com.team4.robot.constants;

import com.team254.lib.util.Units;

public class ShooterConstants {
    public static final int kMasterMotorId = 6;
    public static final int kSlaveMotorId = 7;
    
    public static final double kShooterHeight = .1;
    public static final double kShooterAngle = Math.PI/4;

    public static final double kTargetInMeters = Units.inches_to_meters(TargetingConstants.kFloorToTarget);

    public static final double g = 9.81;
    public static final double dragCoeff = 7.062675;

    public static final double drag = ((dragCoeff * 1.117 * Math.PI * (Math.pow(.0889, 2)/2)));
    public static final double mass = .15;

    public static final double kShooterFlyWheelRadius = Units.inches_to_meters(4);

    public static final double kShooterMaxSpeed = 40;

    public static final double kShooterEnconderPPR = 4096.0;

}