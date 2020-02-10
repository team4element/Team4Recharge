package com.team4.robot.actions;

import com.team4.lib.actionbase.RunOnceAction;
import com.team4.robot.subsystems.Intake;
import com.team4.robot.subsystems.states.IntakeState;

public class DropIntakeAction extends RunOnceAction{
    @Override
    public void runOnce() {
        Intake.getInstance().setControlState(IntakeState.DROP);
    }
}