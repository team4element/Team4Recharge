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
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.SynchronousPIDF;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.drivers.MotorChecker;
import com.team4.lib.drivers.NavX;
import com.team4.lib.drivers.TalonSRXChecker;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Kinematics;
import com.team4.lib.util.Subsystem;
import com.team4.robot.RobotState;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;
import com.team4.robot.constants.DriveConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static Drive mInstance = new Drive();
    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster;
    private final VictorSPX mLeftSlaveA, mLeftSlaveB, mRightSlaveA, mRightSlaveB;
    
    //Path Following
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    // Control states
    private DriveControlState mDriveControlState;
    private TalonControlState mTalonControlState;
    // private DriveCurrentLimitState mDriveCurrentLimitState;    
    private NavX mNavX;
    
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private Rotation2d mTargetHeading = new Rotation2d();
    private boolean mIsOnTarget = false;

    
    private int mInitLeftPosition;
    private int mInitRightPosition;
    private int mInitTicksNeeded;
    private int mInitLeftPositionCorrection;
    private int mInitRightPositionCorrection;


    private boolean mFirstSteerHelper = true;
    private double mSteerPower;

    private SynchronousPIDF mSteerPID;

    // private double mLastDriveCurrentSwitchTime = -1;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                stop();
                setBrakeMode(true);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                // handleFaults();
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case POSITION_SETPOINT:
                        break;
                    case VELOCITY_SETPOINT:
                        break;
                    case PATH_FOLLOWING:
                        if (mPathFollower != null) {
                            updatePathFollower(timestamp);
                        }
                        break;
                    case TURN_TO_HEADING:
                        updateTurnToHeading(timestamp);
                        break;
                    default:
                        System.out.println("unexpected drive control state: " + mDriveControlState);
                        break;
                }

                // if (Superstructure.getInstance().isAtDesiredState()) {
                    // setDriveCurrentState(DriveCurrentLimitState.UNTHROTTLED, timestamp);
                // } else {
                    // setDriveCurrentState(DriveCurrentLimitState.THROTTLED, timestamp);
                // }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            stopLogging();
        }
    };

    private void configureMaster(TalonSRX talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        talon.setInverted(!left);
        talon.setSensorPhase(true);
        // talon.enableVoltageCompensation(true);
        // talon.configVoltageCompSaturation(12.0, IOConstants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(AutoConstants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = CANSpeedControllerFactory.createDefaultTalonSRX(DriveConstants.kLeftDriveMasterId);
        configureMaster(mLeftMaster, true);

        mLeftSlaveA = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kLeftDriveSlaveAId, mLeftMaster);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kLeftDriveSlaveBId, mLeftMaster);
        mLeftSlaveB.setInverted(false);

        mRightMaster = CANSpeedControllerFactory.createDefaultTalonSRX(DriveConstants.kRightDriveMasterId);
        configureMaster(mRightMaster, false);

        mRightSlaveA = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kRightDriveSlaveAId, mRightMaster);
        mRightSlaveA.setInverted(true);

        mRightSlaveB = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kRightDriveSlaveBId, mRightMaster);
        mRightSlaveB.setInverted(true);

        mNavX = new NavX();
        mLeftSlaveB.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer.value, 10, 10);


        reloadGains();

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mDriveControlState = DriveControlState.PATH_FOLLOWING;

        // mDriveCurrentLimitState = DriveCurrentLimitState.UNTHROTTLED;
        // setDriveCurrentState(DriveCurrentLimitState.THROTTLED, 0.0);
     }

    public static Drive getInstance() {
        return mInstance;
    }

    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        // public double left_voltage;
        // public double right_voltage;
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public double gyro_heading_deg;
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        // mPeriodicIO.left_voltage = mLeftMaster.getMotorOutputVoltage();
        // mPeriodicIO.right_voltage = mRightMaster.getMotorOutputVoltage();

        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mNavX.getFusedHeading()).rotateBy(mGyroOffset);
        mPeriodicIO.gyro_heading_deg = mNavX.getFusedHeading();

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / DriveConstants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.left_distance += deltaLeftTicks * (DriveConstants.kDriveWheelDiameterInches * DriveConstants.kDriveWheelGearRatio);

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / DriveConstants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.right_distance += deltaRightTicks * (DriveConstants.kDriveWheelDiameterInches * DriveConstants.kDriveWheelGearRatio);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // if (mDriveControlState == DriveControlState.OPEN_LOOP) {
        if(mDriveControlState == DriveControlState.PATH_FOLLOWING){
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        }
        else if(mTalonControlState == TalonControlState.OPEN || mDriveControlState == DriveControlState.OPEN_LOOP){
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        } else if (mTalonControlState == TalonControlState.MOTION_MAGIC){
            mLeftMaster.set(ControlMode.MotionMagic, (int)mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.MotionMagic, (int)mPeriodicIO.right_demand);
        } else if (mTalonControlState == TalonControlState.POSITION) {
            mLeftMaster.set(ControlMode.Position, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.Position, mPeriodicIO.right_demand);
        } else if (mTalonControlState == TalonControlState.VELOCITY) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.left_feedforward + AutoConstants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / (DriveConstants.kDriveEncoderPPR/4.0));
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.right_feedforward + AutoConstants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / (DriveConstants.kDriveEncoderPPR/4.0));
        } else { // Default Case Velocity
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            // mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                // mPeriodicIO.left_feedforward + GainConstants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / (IOConstants.kDriveEncoderPPR/4.0));
            // mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                // mPeriodicIO.right_feedforward + GainConstants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / (IOConstants.kDriveEncoderPPR/4.0));
    
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (DriveConstants.kDriveWheelCircumferenceInches * DriveConstants.kDriveWheelGearRatio);
    }

    private static double inchesToRotations(double inches) {
        return inches / (DriveConstants.kDriveWheelGearRatio * DriveConstants.kDriveWheelCircumferenceInches);
    }

    private static double inchesToRadians(double inches){
        return inchesToRotations(inches) * (2*Math.PI);
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DriveConstants.kDriveEncoderPPR / 10.0;
    }

        /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        if(mTalonControlState != TalonControlState.OPEN){
            configureOpenTalon();
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward){
        setVelocity(signal, feedforward, true);
    }

    /**
     * Configures talons for velocity control for path following
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward, boolean forceChange) {
        // if(forceChange){
            if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
                // We entered a velocity control state.
                // setBrakeMode(true);
                // mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
                // mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
                // mLeftMaster.configNeutralDeadband(0.0, 0);
                // mRightMaster.configNeutralDeadband(0.0, 0);

                mDriveControlState = DriveControlState.PATH_FOLLOWING;
            // }
        }
        // if(mTalonControlState != TalonControlState.VELOCITY){
            // configureVelocityTalon();
        // }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    /**
     * Configures talons for velocity control 
     */
    public synchronized void setVelocityIPS(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.VELOCITY_SETPOINT) {
            // We entered a velocity control state.
            // setBrakeMode(true);
            // mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mLeftMaster.configNeutralDeadband(0.0, 0);
            // mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        }
        if(mTalonControlState != TalonControlState.VELOCITY){
            configureVelocityTalon();
        }

        double leftVel = radiansPerSecondToTicksPer100ms(inchesToRadians(signal.getLeft()));
        double rightVel = radiansPerSecondToTicksPer100ms(inchesToRadians(signal.getRight()));
        
        setVelocity(new DriveSignal(leftVel, rightVel), feedforward, false);

    }

    public synchronized void setPositionMagic(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
            // We entered a velocity control state.
            // if(mTalonControlState != TalonControlState.MOTION_MAGIC){
                //     configureMagicTalon();
            // }
            // setBrakeMode(true);
            // m_driveLeft1.selectProfileSlot(kLowGearPositionControlSlot, 0);
            // m_driveRight1.selectProfileSlot(kLowGearPositionControlSlot, 0);
            
            // System.out.println("Switching to Position");
            
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
        }
        if(mTalonControlState != TalonControlState.MOTION_MAGIC){
            configureMagicTalon();
        }
    
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        // mPeriodicIO.left_feedforward = feedforward.getLeft();
        // mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setPosition(DriveSignal signal) {
        // if(mDriveControlState != DriveControlState.POSITION_SETPOINT){
            // mDriveControlState = DriveControlState.POSITION_SETPOINT;
        // }
        if(mTalonControlState != TalonControlState.POSITION){
            configurePositionTalon();
        }
    
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        // mPeriodicIO.left_feedforward = feedforward.getLeft();
        // mPeriodicIO.right_feedforward = feedforward.getRight();
    }
    
    
    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized double getHeadingDouble(){
        return mPeriodicIO.gyro_heading_deg;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mNavX.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mRightMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DriveConstants.kDriveEncoderPPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DriveConstants.kDriveEncoderPPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DriveConstants.kDriveEncoderPPR);
    }
    
    public double getEncoderDinstance(){
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DriveConstants.kDriveEncoderPPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / DriveConstants.kDriveWheelTrackWidthInches;
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                    new Lookahead(AutoConstants.kMinLookAhead, AutoConstants.kMaxLookAhead, AutoConstants.kMinLookAheadSpeed,
                            AutoConstants.kMaxLookAheadSpeed),
                    AutoConstants.kInertiaSteeringGain, AutoConstants.kPathFollowingProfileKp,
                    AutoConstants.kPathFollowingProfileKi, AutoConstants.kPathFollowingProfileKv,
                    AutoConstants.kPathFollowingProfileKffv, AutoConstants.kPathFollowingProfileKffa,
                    AutoConstants.kPathFollowingProfileKs, AutoConstants.kPathFollowingMaxVel,
                    AutoConstants.kPathFollowingMaxAccel, AutoConstants.kPathFollowingGoalPosTolerance,
                    AutoConstants.kPathFollowingGoalVelTolerance, AutoConstants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized void setWantTurnToHeading(Rotation2d heading) {
        mDriveControlState = DriveControlState.TURN_TO_HEADING;
         // updatePositionSetpoint(getLeftEncoderDistance(), getRightEncoderDistance());
         mInitLeftPosition = mPeriodicIO.left_position_ticks;
         mInitRightPosition = mPeriodicIO.right_position_ticks;
         setPositionMagic(new DriveSignal(mInitLeftPosition, mInitRightPosition));
        //  setPosition(new DriveSignal(mInitLeftPosition, mInitRightPosition));
 
 
         if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
             mTargetHeading = heading;
             mIsOnTarget = false;
 
             final Rotation2d field_to_robot = getHeading();
             final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);
 
             DriveSignal wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
 
             mInitTicksNeeded = (int)(inchesToRotations(wheel_delta.getRight()) * DriveConstants.kDriveEncoderPPR) ;
 
             mInitLeftPositionCorrection = (int)(mInitLeftPosition - mInitTicksNeeded);
             mInitRightPositionCorrection =  (int)(mInitRightPosition + mInitTicksNeeded);

             setPositionMagic(new DriveSignal(mInitLeftPositionCorrection, mInitRightPositionCorrection));
         }
     }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    
    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING) {
            return mIsOnTarget;
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {

                
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                // DriveSignal setpointInTicks = new DriveSignal(radiansPerSecondToTicksPer100ms(inchesToRadians(setpoint.getLeft())), 
                                                                // radiansPerSecondToTicksPer100ms(inchesToRadians(setpoint.getRight())));
                setVelocity(setpoint, new DriveSignal(0, 0));
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    private void updateTurnToHeading(double timestamp) {
        final Rotation2d field_to_robot = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation();

        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        // Check if we are on target
        final double kGoalPosTolerance = .75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(getLeftLinearVelocity()) < kGoalVelTolerance
                && Math.abs(getRightLinearVelocity()) < kGoalVelTolerance) {
        // Use the current setpoint and base lock.
            mIsOnTarget = true;
            setPositionMagic(new DriveSignal(mPeriodicIO.left_position_ticks, mPeriodicIO.right_position_ticks));
            // setPosition(new DriveSignal(mPeriodicIO.left_position_ticks, mPeriodicIO.right_position_ticks));
            return;
        }

        DriveSignal wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));

        int ticksNeeded = (int)(inchesToRotations(wheel_delta.getRight()) * DriveConstants.kDriveEncoderPPR ) ;

        int m_l_cur_pos = mPeriodicIO.left_position_ticks;
        int m_r_cur_pos = mPeriodicIO.right_position_ticks;

        int wantLeftPos = m_l_cur_pos - ticksNeeded;
        int wantRightPos = m_r_cur_pos + ticksNeeded;

        setPositionMagic(new DriveSignal(wantLeftPos, wantRightPos));
        // setPosition(new DriveSignal(wantLeftPos, wantRightPos));
    }

    private void configureOpenTalon(){
        // setBrakeMode(false);
    
        System.out.println("Switching to open loop");
    
        mTalonControlState = TalonControlState.OPEN;
    }

    private void configureVelocityTalon(){

        setBrakeMode(true);
        mLeftMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
        mRightMaster.selectProfileSlot(kHighGearVelocityControlSlot, 0);
    
        mLeftMaster.configClosedloopRamp(0);
        mRightMaster.configClosedloopRamp(0);
    
        mTalonControlState = TalonControlState.VELOCITY;
    
        System.out.println("Switching to velocity");
    }
    
    private void configureMagicTalon(){

        setBrakeMode(true);
        mLeftMaster.selectProfileSlot(kLowGearPositionControlSlot, 0);
        mRightMaster.selectProfileSlot(kLowGearPositionControlSlot, 0);
    
        mLeftMaster.configClosedloopRamp(12/AutoConstants.kDriveVoltageRampRate);
        mRightMaster.configClosedloopRamp(12/AutoConstants.kDriveVoltageRampRate);

        mTalonControlState = TalonControlState.MOTION_MAGIC;

        System.out.println("Switching to Magic");
    }

    private void configurePositionTalon(){

        setBrakeMode(true);
        mLeftMaster.selectProfileSlot(kLowGearPositionControlSlot, 0);
        mRightMaster.selectProfileSlot(kLowGearPositionControlSlot, 0);
        
        mLeftMaster.configClosedloopRamp(12/AutoConstants.kDriveVoltageRampRate);
        mRightMaster.configClosedloopRamp(12/AutoConstants.kDriveVoltageRampRate);
    
        mTalonControlState = TalonControlState.POSITION;
    
        System.out.println("Switching to Position");
      }

    // private void setDriveLimits(int amps) {
    //     // mRightMaster.configPeakCurrentLimit(amps);
    //     // mLeftMaster.configPeakCurrentLimit(amps);
    // }

    // private void setDriveCurrentState(DriveCurrentLimitState desiredState, double timestamp) {
    //     if (desiredState != mDriveCurrentLimitState && (timestamp - mLastDriveCurrentSwitchTime > 1.0)) {
    //         mLastDriveCurrentSwitchTime = timestamp;
    //         System.out.println("Switching drive current limit state: " + desiredState);
    //         if (desiredState == DriveCurrentLimitState.THROTTLED) {
    //             // setDriveLimits(GainConstants.kDriveCurrentThrottledLimit);
    //         } else {
    //             // setDriveLimits(GainConstants.kDriveCurrentUnThrottledLimit);
    //         }
    //         mDriveCurrentLimitState = desiredState;
    //     }
    // }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }


    public void reloadGains(){
        mLeftMaster.config_kP(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
  
        mRightMaster.config_kP(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kHighGearVelocityControlSlot, AutoConstants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
  
        mLeftMaster.config_kP(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionIZone, Constants.kLongCANTimeoutMs);
        mLeftMaster.configMotionCruiseVelocity((int)radiansPerSecondToTicksPer100ms(AutoConstants.kDriveLowGearMaxVelocity*(DriveConstants.kDriveWheelCircumferenceInches * DriveConstants.kDriveWheelGearRatio)/60));
        mLeftMaster.configMotionAcceleration((int)radiansPerSecondToTicksPer100ms(AutoConstants.kDriveLowGearMaxAccel*(DriveConstants.kDriveWheelCircumferenceInches * DriveConstants.kDriveWheelGearRatio)/60));
  
  
        mRightMaster.config_kP(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearPositionControlSlot, AutoConstants.kDriveLowGearPositionIZone, Constants.kLongCANTimeoutMs);
        mRightMaster.configMotionCruiseVelocity((int)radiansPerSecondToTicksPer100ms(AutoConstants.kDriveLowGearMaxVelocity*(DriveConstants.kDriveWheelCircumferenceInches * DriveConstants.kDriveWheelGearRatio)/60));
        mRightMaster.configMotionAcceleration((int)radiansPerSecondToTicksPer100ms(AutoConstants.kDriveLowGearMaxAccel*(DriveConstants.kDriveWheelCircumferenceInches * DriveConstants.kDriveWheelGearRatio)/60));
  
    }

    private double updateSteerTurn(){
        if(mFirstSteerHelper){
            mSteerPID.setSetpoint(getHeadingDouble());
            mSteerPower = mSteerPID.calculate(getHeadingDouble());

            mFirstSteerHelper = false;
            
        }
        
            mSteerPower = mSteerPID.calculate(getHeadingDouble());
        
        return mSteerPower;
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        TURN_TO_HEADING,
        VELOCITY_SETPOINT,
        POSITION_SETPOINT
    }

    public enum TalonControlState{
        OPEN,
        VELOCITY,       
        MOTION_MAGIC,
        POSITION
  }

    // public enum DriveCurrentLimitState {
    //     UNTHROTTLED, THROTTLED
    // }

    
    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean leftSide = TalonSRXChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                private static final long serialVersionUID = 3643247888353037677L;

                {
                    add(new MotorChecker.MotorConfig<>("left_master", mLeftMaster));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = 2;
                    mRPMFloor = 1500;
                    mCurrentEpsilon = 2.0;
                    mRPMEpsilon = 250;
                    mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
               }
            });
        boolean rightSide = TalonSRXChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                private static final long serialVersionUID = -1212959188716158751L;

                {
                    add(new MotorChecker.MotorConfig<>("right_master", mRightMaster));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = 2;
                    mRPMFloor = 1500;
                    mCurrentEpsilon = 2.0;
                    mRPMEpsilon = 250;
                    mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
               }
            });

        return leftSide && rightSide;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        // SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        // SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());
        SmartDashboard.putNumber("Left Demand", mPeriodicIO.left_demand);
        SmartDashboard.putNumber("Right Demand", mPeriodicIO.right_demand);
        // SmartDashboard.putNumber("X Error", mPeriodicIO.error.getTranslation().x());
        // SmartDashboard.putNumber("Y error", mPeriodicIO.error.getTranslation().y());
        // SmartDashboard.putNumber("Theta Error", mPeriodicIO.error.getRotation().getDegrees());

        // SmartDashboard.putNumber("Left Voltage Kf", mPeriodicIO.left_voltage / getLeftLinearVelocity());
        // SmartDashboard.putNumber("Right Voltage Kf", mPeriodicIO.right_voltage / getRightLinearVelocity());

        // if (mPathFollower != null) {
        //     SmartDashboard.putNumber("Drive LTE", mPathFollower.getAlongTrackError());
        //     SmartDashboard.putNumber("Drive CTE", mPathFollower.getCrossTrackError());
        // } else {
        //     SmartDashboard.putNumber("Drive LTE", 0.0);
        //     SmartDashboard.putNumber("Drive CTE", 0.0);
        // }

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }


}

