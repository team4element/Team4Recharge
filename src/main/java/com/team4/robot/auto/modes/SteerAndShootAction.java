package com.team4.robot.auto.modes;

import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.AutoModeSelector.StartingPosition;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.TurnToHeadingAction;

import edu.wpi.first.wpilibj.DriverStation;

public class SteerAndShootAction extends AutoModeBase{
    
    private StartingPosition mStartPosition;

    public SteerAndShootAction(StartingPosition pose){
        mStartPosition = pose;
    }

    private Rotation2d getAngle(){
        switch(mStartPosition){
            case MID:
                return Rotation2d.fromDegrees(0d);
            case RIGHT:
                return Rotation2d.fromDegrees(45d);
            case LEFT:
                return Rotation2d.fromDegrees(-45d);
            default:
                break;
            }
            DriverStation.reportError("No Start Position Selected", false);
            return null;
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
    
        runAction(new TurnToHeadingAction(getAngle(), 1, true));

        runAction(new AutoSteerAndDistanceAction(200, 3));
        runAction(new ShootAction(5));

        runAction(new WaitAction(15));
    }
}