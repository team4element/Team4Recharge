package com.team4.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    protected TalonFXControlMode mLastFXControlMode = null;

    
    public LazyTalonFX(int deviceNumber) {
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

    @Override
    public void set(TalonFXControlMode mode, double value){
        if (value != mLastSet || mode != mLastFXControlMode) {
            mLastSet = value;
            mLastFXControlMode = mode;
            super.set(mode, value);
        }
    }

}