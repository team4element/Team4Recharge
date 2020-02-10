package com.team4.robot.actions;

import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Intake;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.states.superstructure.SuperstructureState;

public class IntakeThroughPathAction implements Action {
    
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private boolean mStopOnFinish;

    public IntakeThroughPathAction(boolean stopOnFinish){
        mStopOnFinish = stopOnFinish;
    } 
    
    @Override
    public void start() {
        if(!Intake.getInstance().getIsDown()){
            Intake.getInstance().setDown();
        }
        mSuperstructure.setControlState(SuperstructureState.Intake_Convey);
        System.out.println("Starting Intake");
    }

    @Override
    public void update() {
        
    }

    @Override
    public boolean isFinished() {
        return Drive.getInstance().isDoneWithPath();
    }

    @Override
    public void stop() {
        if(mStopOnFinish){
            mSuperstructure.setControlState(SuperstructureState.IDLE);
        }
        System.out.println("stopping intake");
    }
}