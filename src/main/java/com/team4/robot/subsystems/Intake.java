package com.team4.robot.subsystems;

import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.subsystems.states.IntakeState;

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
            case OPEN_LOOP:
                setOpenLoop(.6);
                break;
            case IDLE:
                setOpenLoop(0);
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

    public void setOpenLoop(double signal){
        if(mCurrentState != IntakeState.OPEN_LOOP || mCurrentState != IntakeState.IDLE){
            mCurrentState = IntakeState.OPEN_LOOP;
        }

        mPeriodicIO.demand = signal;
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



    protected static class PeriodicIO{
        public double demand;
    }

}