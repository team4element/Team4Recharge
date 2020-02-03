package com.team4.robot.actions;

import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SynchronousPIDF;
import com.team4.lib.actionbase.Action;
import com.team4.lib.util.DriveHelper;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.VisionTracker;

import edu.wpi.first.wpilibj.Timer;

public class AutoSteerAndDistanceAction implements Action{

    private final Drive mDrive = Drive.getInstance();
    private final VisionTracker mTracker = VisionTracker.getInstance();

    private final double distanceTolerance = 1;
    private final double angleTolerance = 1;

    private SynchronousPIDF distancePID, anglePID;

    private double mDuration, mDistance, mStartTime;

    /**
     * 
     * @param distance measured in inches
     * @param duration measured in seconds
     */
    public AutoSteerAndDistanceAction(double distance, double duration){
        mDistance = distance;
        mDuration = duration;
        
        distancePID = new SynchronousPIDF(AutoConstants.kLimelightDistanceKp, AutoConstants.kLimelightDistanceKi, AutoConstants.kLimelightDistancekd);
        anglePID = new SynchronousPIDF(AutoConstants.kLimelightAngleKp, AutoConstants.kLimelightAngleKi, AutoConstants.kLimelightAngleKd);
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        distancePID.setSetpoint(mDistance);
        anglePID.setSetpoint(0d);
        mTracker.setVisionEnabled(true);
        mDrive.setBrakeMode(false);
    }

    @Override
    public void update() {
        
        double output = distancePID.calculate(mTracker.getTargetDistance()) * .6;
        double turn = anglePID.calculate(VisionTracker.getInstance().getTargetHorizAngleDev()) * .7;

            // DriveSignal setpoint = Kinematics.inverseKinematics(new Twist2d(-output, 0, -turn));
            DriveSignal setpoint = DriveHelper.getInstance().elementDrive(output, turn, false);
            mDrive.setOpenLoop(setpoint);
    }

    @Override
    public boolean isFinished() {
        if(distancePID.onTarget(distanceTolerance) && anglePID.onTarget(angleTolerance)){
            System.out.println("On Target");
            return true;
        }

        return Timer.getFPGATimestamp() - mStartTime >= mDuration;
    }


    @Override
    public void stop() {
        mDrive.setBrakeMode(true);
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        mTracker.setVisionEnabled(false);
    }
}