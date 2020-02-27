package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team4.lib.drivers.LazyVictorSPX;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.ClimberConstants;
import com.team4.robot.subsystems.states.ClimberControlState;

public class Climber extends Subsystem{
    private static Climber instance = null;

    public static Climber getInstance(){
        if(instance == null){
            instance = new Climber();
        }
        return instance;
    }

    private VictorSPX mClimbMotor, mWinchMotor;
    private ClimberControlState mControlState = ClimberControlState.IDLE;

    private PeriodicIO mPeriodicIO;

    private final Loop mLoop = new Loop() {
    
        @Override
        public void onStart(double timestamp) {
            stop();
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(Climber.this){
                switch(mControlState){
                    case CLIMB_UP:
                        setClimb(1.0);
                        break;
                    case CLIMB_DOWN:
                        setClimb(-1.0);
                        break;
                    case WINCH_FORWARD:
                        setWinch(1.0);
                        break;
                    case WINCH_REVERSED:
                        setWinch(-1.0);
                        break;
                    case IDLE:
                        setClimb(0.0);
                        setWinch(0.0);
                        break;
                    default:
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            
        }
    };

    private Climber(){
        mPeriodicIO = new PeriodicIO();

        mClimbMotor = new LazyVictorSPX(ClimberConstants.kClimbMotor);
        mWinchMotor = new LazyVictorSPX(ClimberConstants.kWinchMotor);
    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public void writePeriodicOutputs() {
        mClimbMotor.set(ControlMode.PercentOutput, mPeriodicIO.climb_demand);
        mWinchMotor.set(ControlMode.PercentOutput, mPeriodicIO.winch_demand);
    }

    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }

    public void setClimb(double demand){

    }

    public void setWinch(double demand){

    }

    @Override
    public void stop() {
    
    }

    @Override
    public void outputTelemetry() {
    
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO{
        public double climb_demand;
        public double winch_demand;
    }
}