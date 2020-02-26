package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team4.lib.drivers.LazyTalonSRX;
import com.team4.lib.drivers.LazyVictorSPX;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.subsystems.states.ConveyorControlState;
import com.team4.robot.constants.ConveyorConstants;

public class Conveyor extends Subsystem{
    private static Conveyor mInstance = null;

    private ConveyorControlState mCurrentState = ConveyorControlState.IDLE;

    private VictorSPX mFirstStageRightMotor, mHopperMotor, mFinalStageTopMotor;
    private TalonSRX mFirstStageLeftMotor, mFinalStageBottomMotor;

    private PeriodicIO mPeriodicIO;

    private final Loop mLoop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            stop();
        }
    
        @Override
        public void onLoop(double timestamp) {
            switch(mCurrentState){
                case MOVE_FINAL_STAGE:
                    setFinalStageOnly(-1);
                    break;
                case MOVE_FIRST_STAGE:
                    setFirstStageOnly(1);
                    break;
                case MOVE_FINAL_UNJAM:
                    setFinalStage(-1);
                    setHopper(1);
                    // setFirstStage(0);
                    break;
                case MOVE_FIRST_UNJAM:
                    setFirstStage(1);
                    setHopper(1);
                    // setFinalStage(0);
                    break;
                case MOVE_ALL_STAGES:
                    setFirstStage(1);
                    setHopper(1);
                    setFinalStage(-1);
                    break;
                case IDLE:
                    setFinalStageOnly(0);
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
        mFinalStageBottomMotor = new LazyTalonSRX(ConveyorConstants.kFinalStageBottomMotor);
        mFinalStageTopMotor = new LazyVictorSPX(ConveyorConstants.kFinalStageTopMotor);
        mFinalStageBottomMotor.setInverted(true);
        mFinalStageTopMotor.setInverted(false);

        mHopperMotor = new LazyVictorSPX(ConveyorConstants.kHopperMotor);

        mFirstStageLeftMotor = new LazyTalonSRX(ConveyorConstants.kFirstStageLeftMotor);
        mFirstStageRightMotor = new LazyVictorSPX(ConveyorConstants.kFirstStageRightMotor);

        mFirstStageRightMotor.follow(mFirstStageLeftMotor);
        mFirstStageRightMotor.setInverted(false);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void writePeriodicOutputs() {
        mFirstStageLeftMotor.set(ControlMode.PercentOutput, mPeriodicIO.first_demand);
        mHopperMotor.set(ControlMode.PercentOutput, mPeriodicIO.unjam_demand);
        mFinalStageBottomMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_demand);
        mFinalStageTopMotor.set(ControlMode.PercentOutput, mPeriodicIO.final_demand);
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
    
    public void setFinalStage(double pow){
        mPeriodicIO.final_demand = pow;
    }

    private void setFinalStageOnly(double pow){
        mPeriodicIO.first_demand = 0;
        mPeriodicIO.unjam_demand = 0;
        setFinalStage(pow);
    }

    private void setFirstStage(double pow){
        mPeriodicIO.first_demand = pow;
    }

    private void setHopper(double pow){
        mPeriodicIO.unjam_demand = pow;
    }

    public void setHopperOnly(double pow){
        mPeriodicIO.first_demand = 0;
        mPeriodicIO.final_demand = 0;
        setHopper(pow);
    }

    public void setFirstStageOnly(double pow){
        mPeriodicIO.unjam_demand = 0;
        mPeriodicIO.final_demand = 0;
        setFirstStage(pow);
    }

    public void setControlState(ConveyorControlState state){
        mCurrentState = state;
    }

    @Override
    public void stop() {
        mPeriodicIO.first_demand = 0;
        mPeriodicIO.final_demand = 0;
        mPeriodicIO.unjam_demand = 0;
    }

    protected static class PeriodicIO{
        public double final_demand;
        public double first_demand;
        public double unjam_demand;
    }
}