package com.team4.robot.subsystems;

import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.subsystems.signals.IntakeSignal;

public class Intake extends Subsystem{
    private static Intake mInstance = null;

    private IntakeState mCurrentState = IntakeState.IDLE;

    private PeriodicIO mPeriodicIO;

    public static Intake getInstance(){
        if(mInstance == null){
            mInstance = new Intake();
        }
        return mInstance;
    }

    private Intake(){
        mPeriodicIO = new PeriodicIO();
    }

    private final Loop mLoop = new Loop(){
      public void onStart(double timestamp){

      }  
      public void onLoop(double timestamp){
        switch(mCurrentState){
            case IDLE:
                break;
            default:
                break;
        }
      }
      public void onStop(double timestamp){

      }
    };

    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }

    @Override
    public void readPeriodicInputs() {
    
    }

    @Override
    public void writePeriodicOutputs() {
        //add motor outputs
    }

    public void setOpenLoop(IntakeSignal signal){
        if(mCurrentState != IntakeState.OPEN_LOOP || mCurrentState != IntakeState.IDLE){
            mCurrentState = IntakeState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }

    @Override
    public void outputTelemetry() {
        
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
    
    @Override
    public void stop() {
        
    }

    public enum IntakeState{
        OPEN_LOOP,
        IDLE
    } 

    protected static class PeriodicIO{
        public double left_demand;
        public double right_demand;
    }

}