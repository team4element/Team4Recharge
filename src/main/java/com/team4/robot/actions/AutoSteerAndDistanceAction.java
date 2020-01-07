package com.team4.robot.actions;

import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SynchronousPIDF;
import com.team4.lib.actionbase.Action;
import com.team4.lib.util.Kinematics;
import com.team4.robot.constants.TargetingConstants;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.VisionTracker;

import edu.wpi.first.wpilibj.Timer;

public class AutoSteerAndDistanceAction implements Action{

    private final Drive mDrive = Drive.getInstance();
    private final VisionTracker mTracker = VisionTracker.getInstance();

    private double setpointTolerance = 1;
    ;

    private SynchronousPIDF pid;

    private double mDuration, mDistance, mStartTime;

    public AutoSteerAndDistanceAction(double distance, double duration){
        mDistance = distance;
        mDuration = duration;
        
        pid = new SynchronousPIDF(TargetingConstants.kLimelightKp, TargetingConstants.kLimelightki, TargetingConstants.kLimelightkd);
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        pid.setSetpoint(mDistance);
        mTracker.setVisionEnabled(true);
        mDrive.setBrakeMode(true);
    }

    @Override
    public void update() {
        
        double output = pid.calculate(mTracker.getTargetDistance()) * .3;
        if(mTracker.getTargetDistance() > 0 && mTracker.getTargetDistance() < mDistance && mTracker.getTargetDistance() != 13.790420088692004 /** found when a2 is null */){
            // output = 0;
        }

            double turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.001, 0.1), -0.1);
      
            if(!mTracker.isTargetFound()){
                mDrive.setOpenLoop(new DriveSignal(.1,.1));
            }

            DriveSignal setpoint = Kinematics.inverseKinematics(new Twist2d(-output, 0, -turn));
            mDrive.setOpenLoop(setpoint);
    }

    @Override
    public boolean isFinished() {
        if(pid.onTarget(setpointTolerance) /*&& VisionTracker.getInstance().getTargetHorizAngleDev() <= setpointTolerance*/){
            System.out.println("On Target");
            return true;
        }

        return Timer.getFPGATimestamp() - mStartTime >= mDuration;
    }


    @Override
    public void stop() {
        // mDrive.setOpenLoop(DriveSignal.BRAKE);;
        mTracker.setVisionEnabled(false);
    }
}