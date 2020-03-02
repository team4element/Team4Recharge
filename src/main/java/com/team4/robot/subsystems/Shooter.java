package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team4.lib.drivers.LazyTalonFX;
import com.team4.lib.drivers.TalonUtil;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.ElementMath;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;
import com.team4.robot.constants.ShooterConstants;
import com.team4.robot.subsystems.states.ShooterControlState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem{
    private static Shooter mInstance = null;

    // private TalonSRX mMasterMotor;
    // private VictorSPX mSlaveMotor;

    private TalonFX mMasterMotor, mSlaveMotor;

    private ShooterControlState mControlState = ShooterControlState.IDLE;

    private final Loop mLoop = new Loop(){
        public void onStart(double timestamp){
            stop();
        }
        public void onLoop(double timestamp){
            synchronized(this){
                switch (mControlState){
                    case OPEN_LOOP:
                        setOpenLoop(1);
                        break;
                    case VELOCITY:
                        handleDistanceRPM(VisionTracker.getInstance().getTargetDistance());
                        // setVelocity(3875, 0);
                        break;
                    case IDLE:
                        setOpenLoop(0);
                    default:
                        break;
                }
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
        // mMasterMotor = CANSpeedControllerFactory.createDefaultTalonFX(ShooterConstants.kMasterMotorId);
        mMasterMotor = new LazyTalonFX(ShooterConstants.kMasterMotorId);
        TalonUtil.configureTalonFX(mMasterMotor, true);
        mMasterMotor.changeMotionControlFramePeriod(100);
        mMasterMotor.setControlFramePeriod(ControlFrame.Control_3_General, 10);
        mMasterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                10, Constants.kCANTimeoutMs);
        mMasterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                100, Constants.kCANTimeoutMs);


        // mSlaveMotor = CANSpeedControllerFactory.createDefaultTalonFX(ShooterConstants.kSlaveMotorId);
        mSlaveMotor = new LazyTalonFX(ShooterConstants.kSlaveMotorId);
        TalonUtil.configureTalonFX(mSlaveMotor, false);
        mSlaveMotor.changeMotionControlFramePeriod(100);
        mSlaveMotor.setControlFramePeriod(ControlFrame.Control_3_General, 10);
        mSlaveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                10, Constants.kCANTimeoutMs);
        mSlaveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                100, Constants.kCANTimeoutMs);

        //TODO: test follow mode
        mSlaveMotor.follow(mMasterMotor);


        mMasterMotor.setInverted(TalonFXInvertType.CounterClockwise);
        mSlaveMotor.setInverted(TalonFXInvertType.CounterClockwise);

        setBrakeMode(false);

        mPeriodicIO = new PeriodicIO();

        mControlState = ShooterControlState.IDLE;

        reloadGains();
    }

    

    public synchronized void setBrakeMode(boolean on) {
            TalonUtil.setBrakeMode(mMasterMotor, on);
            TalonUtil.setBrakeMode(mSlaveMotor, on);
        }

    public void setControlState(ShooterControlState state){
        mControlState = state;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.position_ticks = mMasterMotor.getSelectedSensorPosition(0);
       
        mPeriodicIO.velocity = mMasterMotor.getSelectedSensorVelocity(0);
        // mPeriodicIO.velocity = ElementMath.tickPer100msToScaledRPM(mMasterMotor.getSelectedSensorVelocity(0), ShooterConstants.kShooterEnconderPPR, ShooterConstants.kShooterGearRatio);

    }

    @Override
    public void writePeriodicOutputs() {
        if(mControlState == ShooterControlState.OPEN_LOOP){
            mMasterMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
            mSlaveMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
        }else if (mControlState == ShooterControlState.VELOCITY){
            mMasterMotor.set(TalonFXControlMode.Velocity, mPeriodicIO.demand);
            mSlaveMotor.set(TalonFXControlMode.Velocity, mPeriodicIO.demand);
            
        }else{ //force default Open Loop
            mMasterMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
            mSlaveMotor.set(TalonFXControlMode.PercentOutput, mPeriodicIO.demand);
        }

    }

    public double getVelocitySetpoint(){
        if(mControlState == ShooterControlState.VELOCITY){
            return mPeriodicIO.demand;
        }else{
            System.out.println("not in velocity ControlMode");
            return Double.NaN;
        }
    }

    public void handleDistanceRPM(double distance){

        double mFlyWheelDistance = (distance);

        double rpm = 0;

        if(mFlyWheelDistance > 90 && mFlyWheelDistance < 270){
                rpm = (0.025533 * Math.pow(mFlyWheelDistance, 2)) + (1.028 * mFlyWheelDistance) + 2655.6707;
            }

        // System.out.println("Actual RPM: " + rpm);
        if(rpm < 5000){
            rpm = ElementMath.scaleRPM(rpm, ShooterConstants.kShooterGearRatio);
        }else{
            rpm = 0;
        }
        // mPeriodicIO.demand = rpm;
        setVelocity(rpm, 0);
    }

    public void setOpenLoop(double pow){
        mPeriodicIO.demand = pow;
    }

    public void setVelocity(double demand, double ff){
        if(mControlState != ShooterControlState.VELOCITY){
            configureVelocityTalon();
            mControlState = ShooterControlState.VELOCITY;
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

    public TalonFX getMasterTalon(){
        return mMasterMotor;
    }

    public TalonFX getSlaveTalon(){
        return mSlaveMotor;
    }

    @Override
    public void outputTelemetry() {
                // Nothing to output
        // SmartDashboard.putString("Current Shooter Mode", mControlState.toString());
        SmartDashboard.putNumber("Shooter Velocity", mPeriodicIO.velocity);
        SmartDashboard.putNumber("Shooter Setpoint", mPeriodicIO.demand);
        // SmartDashboard.putNumber("Get talon error ", mMasterMotor.getClosedLoopError());
    }

    public void reloadGains(){
        mMasterMotor.config_kP(0, AutoConstants.kShooterKp);
        mMasterMotor.config_kI(0, AutoConstants.kShooterKi);
        mMasterMotor.config_kD(0, AutoConstants.kShooterKd);
        mMasterMotor.config_kF(0, AutoConstants.kShooterKf);

        mSlaveMotor.config_kP(0, AutoConstants.kShooterKp);
        mSlaveMotor.config_kI(0, AutoConstants.kShooterKi);
        mSlaveMotor.config_kD(0, AutoConstants.kShooterKd);
        mSlaveMotor.config_kF(0, AutoConstants.kShooterKf);
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
        mSlaveMotor.set(TalonFXControlMode.PercentOutput, 0);    
    }

    private void configureVelocityTalon(){

        // mMasterMotor.setNeutralMode(NeutralMode.Brake);
        mMasterMotor.selectProfileSlot(0, 0);
    
        mMasterMotor.configClosedloopRamp(0);
    

        System.out.println("Switching shooter to velocity");
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