package com.team4.robot.actions;

import com.team4.lib.actionbase.Action;
import com.team4.robot.subsystems.Drive;

public class IntakeThroughPathAction implements Action {
    

    public IntakeThroughPathAction(){
    }
    
    @Override
    public void start() {
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
        System.out.println("stopping intake");
    }
}