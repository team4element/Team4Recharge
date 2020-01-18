package com.team4.lib.util;

import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;

public class DriveHelper{

    private static DriveHelper instance = null;

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
        
        
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        
        return new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor);
    }
}
