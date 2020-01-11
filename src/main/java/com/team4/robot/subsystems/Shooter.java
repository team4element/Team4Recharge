package com.team4.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.util.Units;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.drivers.MotorChecker;
import com.team4.lib.drivers.TalonFXChecker;
import com.team4.lib.drivers.TalonUtil;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.ElementMath;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.ShooterConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem{
    private static Shooter mInstance = null;

    // private TalonSRX mMasterMotor;
    // private VictorSPX mSlaveMotor;

    private TalonFX mMasterMotor, mSlaveMotor;

    private boolean mIsBrakeMode = false;

    private ShooterState mControlState = ShooterState.IDLE;

    private final Loop mLoop = new Loop(){
        public void onStart(double timestamp){
            stop();
        }
        public void onLoop(double timestamp){
            switch (mControlState){
                case OPEN_LOOP:
                    // setOpenLoop(1);
                    break;
                case VELOCITY:
                    // handleDistanceRPM(VisionTracker.getInstance().getTargetDistance());
                    break;
                case IDLE:
                    // setOpenLoop(0);
                default:
                    break;
            }
        }
        public void onStop(double timestamp){
            stop();
        }
    };

    PeriodicIO mPeriodicIO;

    public static Shooter getInstance(){
        if(mInstance == null){
            mInstance = new Shooter();
        }
        return mInstance;
    }

    

    private Shooter(){
        mMasterMotor = CANSpeedControllerFactory.createDefaultTalonFX(ShooterConstants.kMasterMotorId);
        TalonUtil.configureTalonFX(mMasterMotor, true);

        mSlaveMotor = CANSpeedControllerFactory.createPermanentSlaveTalonFX(ShooterConstants.kSlaveMotorId, mMasterMotor);
        TalonUtil.configureTalonFX(mSlaveMotor, false);

        // mSlaveMotor.setInverted(false);

        setBrakeMode(false);

        mPeriodicIO = new PeriodicIO();

        mControlState = ShooterState.IDLE;

    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mMasterMotor.setNeutralMode(mode);
            mSlaveMotor.setNeutralMode(mode);
        }
    }

    public void setControlState(ShooterState state){
        mControlState = state;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.position_ticks = mMasterMotor.getSelectedSensorPosition(0);
       
        mPeriodicIO.velocity = ElementMath.tickPer100msToScaledRPM(mMasterMotor.getSelectedSensorVelocity(0), ShooterConstants.kShooterEnconderPPR, ShooterConstants.kShooterGearRatio);

    }

    @Override
    public void writePeriodicOutputs() {
        if(mControlState == ShooterState.OPEN_LOOP){
            mMasterMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
            mSlaveMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
        }else if (mControlState == ShooterState.VELOCITY){
            mMasterMotor.set(TalonFXControlMode.Velocity, mPeriodicIO.demand);
            //TODO: test with ArbitraryFeedForward
            // mMasterMotor.set(TalonFXControlMode.Velocity, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
            // mPeriodicIO.feedforward + AutoConstants.kShooterKd * mPeriodicIO.accel / (ShooterConstants.kShooterEnconderPPR/4.0));
        }else{ //force default Open Loop
            mMasterMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
            mSlaveMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
        }

    }

    public void handleDistanceRPM(double distance){

        double distanceInMeters = Units.inches_to_meters(distance); 
        boolean isFirstCorrectVel = true; 

        double firstCorrectVel = 0;
        double mPrevVel = 0;

        for(double i=4; i <=ShooterConstants.kShooterMaxSpeed; i = i + .05 ){
            for(double j = 0; j <= 6; j = j + .05){
                    double t = j;

                    double x = (ShooterConstants.kMass / ShooterConstants.kDrag) * 
                    i * Math.cos(ShooterConstants.kShooterAngle) * 
                    (1 - Math.pow(Math.E, (-(ShooterConstants.kDrag*t)/ShooterConstants.kMass)));

                    double y = ((-(ShooterConstants.kMass*ShooterConstants.kGravityConstant) * t)/ 
                    ShooterConstants.kDrag) + (ShooterConstants.kMass/ShooterConstants.kDrag) *
                    (i*Math.sin(ShooterConstants.kShooterAngle)+(ShooterConstants.kMass*ShooterConstants.kGravityConstant)/ShooterConstants.kDrag) * 
                    (1 - Math.pow(Math.E, (-(ShooterConstants.kDrag*t)/ShooterConstants.kMass))) + ShooterConstants.kShooterHeight;

                    if (y < 0){
                        break;
                    }

                    if(Math.abs(x - distanceInMeters) < .2 && Math.abs(y - ShooterConstants.kTargetInMeters) < .2){
                        if(isFirstCorrectVel){
                            firstCorrectVel = i;
                            isFirstCorrectVel = false;
                            // System.out.println("x: " + x + "y: " + y + "velocity: " + firstCorrectVel);
                        }
                        if(mPrevVel <= i){
                            // System.out.println("x: " + x + "y: " + y + "velocity: " + i);
                            mPrevVel = i;
                        }        
                    }


            }
        }
        double low_speed = firstCorrectVel;
        double high_speed = mPrevVel;  
        

        double low_speed_rpm = findRPM(low_speed);
        double high_speed_rpm = findRPM(high_speed);

        double rpm = Math.abs(low_speed_rpm + high_speed_rpm)/2;

        rpm = ElementMath.scaleRPM(rpm, ShooterConstants.kShooterGearRatio);
        // mPeriodicIO.demand = rpm;
        setVelocity(rpm, 0);
        // System.out.println(rpm);
    }

    public double findRPM(double mps){
        double t_speed = 2 * mps;
        double rpm = (60*t_speed)/(2*Math.PI*ShooterConstants.kShooterFlyWheelRadius);
        return rpm;
    }

    public void setOpenLoop(double pow){
        if(mControlState != ShooterState.OPEN_LOOP || mControlState != ShooterState.IDLE){
            mControlState = ShooterState.OPEN_LOOP;
        }
        mPeriodicIO.demand = pow;
    }

    public void setVelocity(double demand, double ff){
        if(mControlState != ShooterState.VELOCITY){
            configureVelocityTalon();
            mControlState = ShooterState.VELOCITY;
        }


        mPeriodicIO.demand = ElementMath.rpmToTicksPer100ms(demand, ShooterConstants.kShooterEnconderPPR);
        mPeriodicIO.feedforward = ff;
    }

    @Override
    public void zeroSensors() {
        resetEncoders();
    }

    public synchronized void resetEncoders() {
        mMasterMotor.setSelectedSensorPosition(0, 0, 0);
        mSlaveMotor.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public boolean checkSystem() {
        boolean master = TalonFXChecker.checkMotors(this,
        new ArrayList<MotorChecker.MotorConfig<TalonFX>>() {
            private static final long serialVersionUID = 3643247888353037677L;

            {
                add(new MotorChecker.MotorConfig<>("Master", mMasterMotor));
                add(new MotorChecker.MotorConfig<>("Slave", mSlaveMotor));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mCurrentFloor = 2;
                mRPMFloor = 1600;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 250;
                mRPMSupplier = () -> mMasterMotor.getSelectedSensorVelocity(0);
           }
        });
        return master;
    }

    @Override
    public void outputTelemetry() {
        // Nothing to output
        SmartDashboard.putString("Current Shooter Mode", mControlState.toString());
        SmartDashboard.putNumber("Shooter Velocity in RPM", mPeriodicIO.velocity);
    }

    @Override
    public double getVelocity() {
        return mPeriodicIO.velocity;
    }
    
    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }
    
    @Override
    public void stop() {
        mMasterMotor.set(TalonFXControlMode.PercentOutput, 0);    
    }

    private void configureVelocityTalon(){

        mMasterMotor.setNeutralMode(NeutralMode.Brake);
        mMasterMotor.selectProfileSlot(0, 0);
    
        mMasterMotor.configClosedloopRamp(0);
    

        System.out.println("Switching shooter to velocity");
    }

    public enum ShooterState{
        OPEN_LOOP,
        VELOCITY,
        IDLE
    }

    protected static class PeriodicIO{
        public double timestamp;
        public double demand;
        public double feedforward;
        public double position_ticks;
        public double accel;
        public double velocity;
    }
}