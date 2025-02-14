package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.Constants;
import com.team4.robot.constants.IntakeConstants;
import com.team4.robot.subsystems.states.IntakeState;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem{
    private static Intake mInstance = null;

    private IntakeState mCurrentState = IntakeState.IDLE;

    private TalonSRX mMotor;
    
    private boolean mIsDown = false;
    
    private Solenoid mLeftPiston, mRightPiston;

    private PeriodicIO mPeriodicIO;

    public static Intake getInstance(){
        if(mInstance == null){
            mInstance = new Intake();
        }
        return mInstance;
    }

    private Intake(){
   
        mMotor = CANSpeedControllerFactory.createDefaultTalonSRX(IntakeConstants.kIntakeMotor);
        mMotor.setInverted(true);

        mMotor.changeMotionControlFramePeriod(100);
        mMotor.setControlFramePeriod(ControlFrame.Control_3_General, 20);
        mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                20, Constants.kCANTimeoutMs);
        mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                100, Constants.kCANTimeoutMs);


        mLeftPiston = new Solenoid(1, IntakeConstants.kLeftSolenoidId);
        mRightPiston = new Solenoid(1, IntakeConstants.kRightSolenoidId);

        mPeriodicIO = new PeriodicIO();
    }


    private final Loop mLoop = new Loop(){
      public void onStart(double timestamp){

      }  
      public void onLoop(double timestamp){
        switch(mCurrentState){
            case OPEN_LOOP:
                setOpenLoop(.4);
                break;
            case REVERSE:
                setOpenLoop(-.4);
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
        mMotor.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    public void setOpenLoop(double signal){
        if(mCurrentState != IntakeState.OPEN_LOOP || mCurrentState != IntakeState.IDLE){
            mCurrentState = IntakeState.OPEN_LOOP;
        }

        // if(mIsDown && signal != 0){
            mPeriodicIO.demand = signal;
        // }else{
            // mPeriodicIO.demand = 0;
        // }
    }

    public void setControlState(IntakeState state){
        if(mCurrentState != state){
            mCurrentState = state;
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Is Intake Down", mIsDown);
    }



    @Override
    public void stop() {
        
    }

    public void setDown(){
        if(!mIsDown){
            mLeftPiston.set(true);
            mRightPiston.set(true);


            mIsDown = true;
        }
    }

    public void setUp(){
        if(mIsDown){
            mLeftPiston.set(false);
            mRightPiston.set(false);

            mIsDown = false;
        }
    }

    public boolean getIsDown(){
        return mIsDown;
    }

    protected static class PeriodicIO{
        public double demand;
    }

}