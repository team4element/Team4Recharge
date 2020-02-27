package com.team4.robot.subsystems;

import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.subsystems.WheelHandler.CurrentWheelMode;
import com.team4.robot.subsystems.states.ConveyorControlState;
import com.team4.robot.subsystems.states.IntakeState;
import com.team4.robot.subsystems.states.ShooterControlState;
import com.team4.robot.subsystems.states.superstructure.SuperstructureState;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The purpose of the Superstructure subsystem is to handle multiple elements of the robot's superstructure at once.
 * Instead of just moving the Shooter and Conveyor separately, the goal is that it handle that for us so we have no need to alter it.
 * 
 * This deals with vision handeling to allow for driver ease.
 * 
 */

public class Superstructure extends Subsystem{
    private static Superstructure instance = null;

    private boolean mShooterDip = true;

    private int mShootCount = 0;

    public static Superstructure getInstance(){
        if(instance == null){
            instance = new Superstructure();
        }
        return instance;
    }

    private SuperstructureState mControlState = SuperstructureState.IDLE;

    private final Loop mLoop = new Loop(){
        public void onStart(double timestamp){
            synchronized(Superstructure.this){

            }
        }
        public void onLoop(double timestamp){
            synchronized(Superstructure.this){
                switch(mControlState){
                    case IDLE:
                        mShooter.setControlState(ShooterControlState.IDLE);
                        mConveyor.setControlState(ConveyorControlState.IDLE);
                        mIntake.setControlState(IntakeState.IDLE);
                        mWheelHandler.setWheelControlState(CurrentWheelMode.IDLE);
                        break;
                    case Convey_Shoot:
                        handleConveyAndShoot();
                        break;
                    case Intake_Convey:
                        handleIntakeAndConvey();
                        break;
                    case Convey_Convey:
                        handleConveyAndConvey();
                        break;
                    case Shoot:
                        mIntake.setControlState(IntakeState.IDLE);
                        mShooter.setControlState(ShooterControlState.VELOCITY);
                        break;
                    case Enable_Wheel_Rotation:
                        handleWheelRotation();
                        break;
                    case Enable_Wheel_Position:
                        handleWheelPosition();
                        break;
                    default:
                        DriverStation.reportError("In an Invalid Superstructure State", false);
                }
            }
        }

        public void onStop(double timestamp){
            stop();
        }
    };

    private final Shooter mShooter = Shooter.getInstance();
    private final Conveyor mConveyor = Conveyor.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final WheelHandler mWheelHandler = WheelHandler.getInstance();

    private Superstructure(){
        
    }

    public synchronized void handleConveyAndConvey(){
        mConveyor.setControlState(ConveyorControlState.MOVE_FINAL_STAGE);
    }


    public synchronized void handleIntakeAndConvey(){
        // if(mIntake.getIsDown()){
            mIntake.setControlState(IntakeState.OPEN_LOOP);
        // }else{
            // mIntake.setControlState(IntakeState.IDLE);
        // }

        mConveyor.setControlState(ConveyorControlState.MOVE_FIRST_STAGE);
        
    }

    public synchronized void handleConveyAndShoot(){
        mIntake.setControlState(IntakeState.IDLE);
        mShooter.setControlState(ShooterControlState.VELOCITY);
        if(mShooter.getVelocity() >= mShooter.getVelocitySetpoint() && mShooter.getVelocitySetpoint() != 0d){
            mConveyor.setControlState(ConveyorControlState.MOVE_FINAL_UNJAM);
        }else{
            mConveyor.setControlState(ConveyorControlState.IDLE);
        }
    }

    public synchronized void handleWheelRotation(){
        mIntake.setControlState(IntakeState.IDLE);
        mWheelHandler.setWheelControlState(CurrentWheelMode.ROTATION);
    }

    public synchronized void handleWheelPosition(){
        mIntake.setControlState(IntakeState.IDLE);
        mWheelHandler.setWheelControlState(CurrentWheelMode.POSITION);
    }

    public synchronized void countShooterVelocity(){
        if(!mShooterDip){
            if(mShooter.getVelocity() <= mShooter.getVelocitySetpoint()){
                mShooterDip = true;
                mShootCount += 1;
            }
        }else{
            if(mShooter.getVelocity() >= mShooter.getVelocitySetpoint()){
                mShooterDip = false;
            }
        }
    }

    public synchronized void resetCount(){
        mShootCount = 0;
        mShooterDip = false;
    }

    public synchronized int getShooterCount(){
        return mShootCount;
    }

    public synchronized void setControlState(SuperstructureState state){
        if(mControlState != state){
            mControlState = state;
        }
    }
    
    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }

    @Override
    public void stop() {
        resetCount();
        mShooter.stop();
        mConveyor.stop();
    }

    @Override
    public void outputTelemetry() {
        
    }

    @Override
    public boolean checkSystem() {
        return false;
    }



}