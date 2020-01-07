package com.team4.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * 254's LazyTalonSRX but for VictorSPX
 */
public class LazyVictorSPX extends VictorSPX{
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }

}
