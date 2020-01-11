package com.team4.robot.subsystems.signals;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class IntakeSignal {
    protected double mLeftMotor;
    protected double mRightMotor;
    protected boolean mBrakeMode;

    public IntakeSignal(double left, double right) {
        this(left, right, false);
    }

    public IntakeSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static IntakeSignal NEUTRAL = new IntakeSignal(0, 0);
    public static IntakeSignal BRAKE = new IntakeSignal(0, 0, true);

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}