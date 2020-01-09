package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team4.lib.drivers.LazyVictorSPX;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.ConveyorConstants;

public class Conveyor extends Subsystem{
    private static Conveyor mInstance = null;

    private VictorSPX mMotor;

    private PeriodicIO mPeriodicIO;

    private final Loop mLoop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            stop();
        }
    
        @Override
        public void onLoop(double timestamp) {
            
        }

        
        @Override
        public void onStop(double timestamp) {
            stop();        
        }
    

    }; 

    public static Conveyor getInstance(){
        if(mInstance == null){
            mInstance = new Conveyor();
        }
        return mInstance;
    }
    

    private Conveyor(){
        mMotor = new LazyVictorSPX(ConveyorConstants.kMotorId);
    
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void writePeriodicOutputs() {
        mMotor.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        //does nothing
    }

    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }
    
    public void setOpenLoop(double pow){
        mPeriodicIO.demand = pow;
    }

    @Override
    public void stop() {
        mPeriodicIO.demand = 0;
    }

    private static class PeriodicIO{
        public double demand;
    }
}