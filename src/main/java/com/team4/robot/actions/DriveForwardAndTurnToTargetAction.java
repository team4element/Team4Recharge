package com.team4.robot.actions;

import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.SynchronousPIDF;
import com.team4.lib.actionbase.Action;
import com.team4.robot.constants.TargetingConstants;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.VisionTracker;

import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAndTurnToTargetAction implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final VisionTracker mLL = VisionTracker.getInstance();

    private static final SynchronousPIDF mPID = new SynchronousPIDF(TargetingConstants.kLimelightKp, TargetingConstants.kLimelightki, TargetingConstants.kLimelightkd);

    private double mStartTime;
    private final double mDuration;
    private boolean mFinished;
    private double mVelocity; 

    private boolean validTarget;
    private double headingError;
    private double steeringAdjust;
    private double lastTime;
    


    // private double minimumCommand = Constants.DRIVE_FEEDFORWARD;

    private double leftCommand = 0.0;
    private double rightCommand = 0.0;

    public  DriveForwardAndTurnToTargetAction(double velocity, double duration) {
        mVelocity = -velocity;
        mDuration = duration;
        lastTime = Timer.getFPGATimestamp();
    }

    public  DriveForwardAndTurnToTargetAction(double velocity, double duration, double heading) {
        mVelocity = velocity;
        mDuration = duration;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration || mFinished; 
    }

    @Override
    public void update() {

        validTarget = mLL.isTargetFound();
        headingError = mLL.getTargetHorizAngleDev();

        double dt = Timer.getFPGATimestamp() - lastTime;

       

        if (validTarget) {

            steeringAdjust = -mPID.calculate(headingError, dt) * mVelocity;

            if(mVelocity < 0)
                steeringAdjust = -steeringAdjust;

            
            System.out.println("Valid Target");
        } else {
            steeringAdjust = 0.0;
            //steeringAdjust = mPID.calculate((heading - mLastValidHeading), dt) * mVelocity;

        }

        // System.out.println("STEERING ADJUST IS " + steeringAdjust + "!!!!!!!!!!!!!");


        // if(steeringAdjust > maxsteeringAdjust){
        //     steeringAdjust = maxsteeringAdjust;
        //     System.out.println("Max Steering Adjust");
        // }
        // if(steeringAdjust < -maxsteeringAdjust){
        // steeringAdjust = -maxsteeringAdjust;
        // System.out.println("Max Steering Adjust");
        // }

        // System.out.println("Steering Adjust: " + steeringAdjust);
        

        leftCommand =  mVelocity + steeringAdjust;
        rightCommand  = mVelocity - steeringAdjust;

        // double highest_vel = Math.max(Math.abs(leftCommand), Math.abs(rightCommand));

        // if(highest_vel > 1.0){
        //     leftCommand /= highest_vel;
        //     rightCommand /= highest_vel;
        // }

        // System.out.println("LEFT COMMAND " + leftCommand + "!!!!!!!!");
        // System.out.println("RIGHT COMMAND " + rightCommand + "!!!!!!!!");

       mDrive.setVelocityInchesPerSecond(new DriveSignal(leftCommand, rightCommand));

        lastTime = Timer.getFPGATimestamp();

    }

    @Override
    public void stop() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mLL.setVisionEnabled(true);
        System.out.println("Starting DriveForwardAndTurnToTarget");
    }
}