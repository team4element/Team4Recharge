package com.team4.robot.actions;

import com.team254.lib.geometry.Pose2d;
import com.team4.lib.actionbase.RunOnceAction;
import com.team4.robot.RobotState;

import edu.wpi.first.wpilibj.Timer;

public class ResetPoseAction extends RunOnceAction{

    private Pose2d mPose;

    public ResetPoseAction(Pose2d pose){
        mPose = pose;
    }

    @Override
    public void runOnce() {
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), mPose);
    }
}