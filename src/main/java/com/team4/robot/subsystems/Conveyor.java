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

    private ConveyorState mCurrentState = ConveyorState.IDLE;

    private VictorSPX mMotor;

    private PeriodicIO mPeriodicIO;

    private final Loop mLoop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            stop();
        }
    
        @Override
        public void onLoop(double timestamp) {
            switch(mCurrentState){
                case FORWARD:
                    setOpenLoop(.6);
                    break;
                case REVERSE:
                    setOpenLoop(-.6);
                    break;
                case IDLE:
                    setOpenLoop(0);
                    break;
                default:
                    break;
            }
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
        //no talons currently to test
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
    
    private void setOpenLoop(double pow){
        mPeriodicIO.demand = pow;
    }

    public void setControlState(ConveyorState state){
        mCurrentState = state;
    }

    @Override
    public void stop() {
        mPeriodicIO.demand = 0;
    }

    public enum ConveyorState{
        IDLE,
        FORWARD,
        REVERSE
    }

    protected static class PeriodicIO{
        public double demand;
    }
}