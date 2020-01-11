package com.team4.lib.util;

import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;

public class DriveHelper{

    private static DriveHelper instance = null;

    private static final double kNegInertiaThreshold = 0.65;
    private static final double kNegInertiaTurnScalar = 3.5;
    private static final double kNegInertiaCloseScalar = 4.0;
    private static final double kNegInertiaFarScalar = 5.0;

    private double mOldWheel = 0.0;
    private double mNegInertiaAccumlator = 0.0;

    public static DriveHelper getInstance(){
        if(instance == null){
            instance = new DriveHelper();
        }
        return instance;
    }

    public DriveHelper(){
    }


    //TODO: scale throttle and turn to optimal amount where driver can control
    public DriveSignal elementDrive(double throttle, double wheel, boolean quickTurn) {
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        final double kThrottleGain = .25;
        final double kThrottleLinearity = 1.5;
        final double throttleDenom = Math.sin(Math.PI / 2.0 * kThrottleLinearity);
        throttle = Math.sin(Math.PI / 2.0 * kThrottleLinearity * throttle);
        throttle = throttle / (throttleDenom * throttleDenom);
        throttle *= kThrottleGain;


        
        
        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator);
        }        
        wheel *= kWheelGain;
        
        
        // Negative inertia!
        double negInertiaScalar;
        if (wheel * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = kNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(wheel) > kNegInertiaThreshold) {
                    negInertiaScalar = kNegInertiaFarScalar;
                } else {
                    negInertiaScalar = kNegInertiaCloseScalar;
                }
            }

            double negInertiaPower = negInertia * negInertiaScalar;
            mNegInertiaAccumlator += negInertiaPower;    

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }

        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        
        return new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor);
    }
}
