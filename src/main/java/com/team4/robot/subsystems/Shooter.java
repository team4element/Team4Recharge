package com.team4.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team254.lib.util.Units;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.drivers.MotorChecker;
import com.team4.lib.drivers.TalonSRXChecker;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.ElementMath;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;
import com.team4.robot.constants.ShooterConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends Subsystem{
    private static Shooter mInstance = null;

    private TalonSRX mMasterMotor;
    private VictorSPX mSlaveMotor;

    private ShooterState mControlState;

    private final Loop mLoop = new Loop(){
        public void onStart(double timestamp){
            stop();
        }
        public void onLoop(double timestamp){
            switch (mControlState){
                case OPEN_LOOP:
                    setShooterOpenLoop(1);
                case VELOCITY:
                    handleDistanceRPM(VisionTracker.getInstance().getTargetDistance());
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

    private void configureMaster(TalonSRX talon) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + ("Motor") + " encoder: " + sensorPresent, false);
        }
        talon.setSensorPhase(true);
        // talon.enableVoltageCompensation(true);
        // talon.configVoltageCompSaturation(12.0, IOConstants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(AutoConstants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private Shooter(){
        mMasterMotor = CANSpeedControllerFactory.createDefaultTalonSRX(ShooterConstants.kMasterMotorId);
        configureMaster(mMasterMotor);

        mSlaveMotor = CANSpeedControllerFactory.createPermanentSlaveVictor(ShooterConstants.kSlaveMotorId, mMasterMotor);

        mSlaveMotor.setInverted(false);

        mPeriodicIO = new PeriodicIO();

        mControlState = ShooterState.VELOCITY;

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        double prevLeftTicks = mPeriodicIO.position_ticks;
        mPeriodicIO.position_ticks = mMasterMotor.getSelectedSensorPosition(0);
       
        mPeriodicIO.velocity = ElementMath.tickPer100msToRPM(mMasterMotor.getSelectedSensorVelocity(0), ShooterConstants.kShooterEnconderPPR);

    }

    @Override
    public void writePeriodicOutputs() {
        if(mControlState == ShooterState.OPEN_LOOP){
            mMasterMotor.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        }else if (mControlState == ShooterState.VELOCITY){
            mMasterMotor.set(ControlMode.Velocity, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
            mPeriodicIO.feedforward + AutoConstants.kShooterKd * mPeriodicIO.accel / (ShooterConstants.kShooterEnconderPPR/4.0));
        }else{

        }

    }

    public void handleDistanceRPM(double distance){

        double distanceInMeters = Units.inches_to_meters(distance); 
        boolean isFirstCorrectVel = true; 

        double firstCorrectVel = 0;
        double mPrevVel = 0;

        for(double i=4; i <=ShooterConstants.kShooterMaxSpeed; i = i + .05 ){
            for(double j = 0; j <= 2.5; j = j + .05){
                    double t = j;

                    double x = (ShooterConstants.mass / ShooterConstants.drag) * 
                    i * Math.cos(ShooterConstants.kShooterAngle) * 
                    (1 - Math.pow(Math.E, (-(ShooterConstants.drag*t)/ShooterConstants.mass)));

                    double y = ((-(ShooterConstants.mass*ShooterConstants.g) * t)/ 
                    ShooterConstants.drag) + (ShooterConstants.mass/ShooterConstants.drag) *
                    (i*Math.sin(ShooterConstants.kShooterAngle)+(ShooterConstants.mass*ShooterConstants.g)/ShooterConstants.drag) * 
                    (1 - Math.pow(Math.E, (-(ShooterConstants.drag*t)/ShooterConstants.mass))) + ShooterConstants.kShooterHeight;

                    if (y < 0){
                        break;
                    }

                    if(Math.abs(x - distanceInMeters) < .1 && Math.abs(y - ShooterConstants.kTargetInMeters) < .1){
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

        mPeriodicIO.demand = rpm;
        setVelocity(rpm, 0);
        // System.out.println(rpm);
    }

    public double findRPM(double mps){
        double t_speed = 2 * mps;
        double rpm = (60*t_speed)/(2*Math.PI*ShooterConstants.kShooterFlyWheelRadius);
        return rpm;
    }

    public void setShooterOpenLoop(double pow){
        if(mControlState != ShooterState.OPEN_LOOP){
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
    public boolean checkSystem() {
        boolean master = TalonSRXChecker.checkMotors(this,
        new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
            private static final long serialVersionUID = 3643247888353037677L;

            {
                add(new MotorChecker.MotorConfig<>("left_master", mMasterMotor));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mCurrentFloor = 2;
                mRPMFloor = 1500;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 250;
                mRPMSupplier = () -> mMasterMotor.getSelectedSensorVelocity(0);
           }
        });
        return master;
    }

    @Override
    public void outputTelemetry() {
        //Nothing to output
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
        mMasterMotor.set(ControlMode.PercentOutput, 0);    
    }

    private void configureVelocityTalon(){

        mMasterMotor.setNeutralMode(NeutralMode.Brake);
        mMasterMotor.selectProfileSlot(0, 0);
    
        mMasterMotor.configClosedloopRamp(0);
    

        System.out.println("Switching to velocity");
    }

    private enum ShooterState{
        OPEN_LOOP,
        VELOCITY
    }

    private static class PeriodicIO{
        public double timestamp;
        public double demand;
        public double feedforward;
        public double position_ticks;
        public double accel;
        public double velocity;
    }
}