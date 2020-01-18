package com.team4.robot.planners;

import com.team254.lib.util.CSVWritable;

public class ShooterMotionPlanner implements CSVWritable{
    
    public enum ProfileType{
        PID, PIDF, NONLINEAR_FEEDBACK, SET_PROFILE
    }

    private ProfileType mProfileType = ProfileType.PIDF;

    public void setProfileType(ProfileType type){
        mProfileType = type;
    }

    //All velocity inputs in RPM
    private double mDesiredVelocity;
    private double mLastTime = Double.POSITIVE_INFINITY;
    private double mError;
    private double mPrevError = 0.0;
    private double mDt;
    private Output mOutput;

    @Override
    public String toCSV() {
        return "Nothing to write";
    }

    public void configSetpoint(double desiredVel){
        mDesiredVelocity = desiredVel;
    }

    public Output update(double timestamp, double currentVel){
        if(mDesiredVelocity == Double.NaN){return new Output();}
        
        if(mDesiredVelocity - currentVel >= mDesiredVelocity - .4 && mDesiredVelocity - currentVel <= mDesiredVelocity + .4 && !Double.isFinite(mLastTime)){
            mLastTime = timestamp;
        }
        
        mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        if(mDesiredVelocity - currentVel >0.0){
            final double velocity = mDesiredVelocity;
            final double acceleration = mDesiredVelocity * 1;
            final double feedforward = (.9*1023)/ mDesiredVelocity;
            
            if(mProfileType == ProfileType.PID){
                mOutput = updatePID(velocity, acceleration, feedforward);
            }else if(mProfileType == ProfileType.PIDF){

            }else if(mProfileType == ProfileType.NONLINEAR_FEEDBACK){

            }else if(mProfileType == ProfileType.SET_PROFILE){

            }
        }else{
            return new Output();
        }



        return mOutput;

    }

    protected Output updatePID(double velocity, double acceleration, double feedforward){
        double kP = .1;
        double kI = 0.0;
        double kD = 0.0;
        
        double accel = acceleration;

        mError = mDesiredVelocity - velocity;

        double P = mError * kP;
        double I = 0;
        I += mError * kI;
        double D = ((mError - mPrevError) / mDt) * kD;
        
        double outputVel = P + I + D;
        
        if(acceleration > outputVel){
            accel = outputVel;
        }
        
        double outputAccel = accel;
        double outputFeedforward = feedforward;


        mPrevError = mError;
        return new Output(outputVel, outputAccel, outputFeedforward);
    }

    public Output updateNonLinear(double currentVel){
        final double kBeta = 2.0;  // >0.
        final double kZeta = 0.7;  // Damping coefficient, [0, 1].

        // Compute gain parameter.
        // final double k = 2.0 * kZeta * Math.sqrt(kBeta * currentVel * currentVel  * currentVel);


        return new Output();
    }

    public void reset(){
        mOutput = new Output();
        mLastTime = Double.POSITIVE_INFINITY;

    }

    public static class Output{

        public Output(){

        }
        public Output(double velocity, double acceleration, double feedforward){
            this.velocity = velocity;
            this.accel = acceleration;
            this.voltage_feedforward = feedforward;
        }

        public double velocity;

        public double accel;

        public double voltage_feedforward;
    }
}