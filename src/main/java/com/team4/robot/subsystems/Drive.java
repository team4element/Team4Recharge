package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.DriveSignal;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.drivers.NavX;
import com.team4.lib.drivers.TalonUtil;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.ElementMath;
import com.team4.lib.util.Kinematics;
import com.team4.lib.util.Subsystem;
import com.team4.robot.RobotState;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;
import com.team4.robot.constants.DriveConstants;
import com.team4.robot.subsystems.states.DriveControlState;
import com.team4.robot.subsystems.states.TalonControlState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static Drive mInstance = new Drive();
    // Hardware
    private final TalonFX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
    
    //Path Following
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    // Control states
    private DriveControlState mDriveControlState;
    private TalonControlState mTalonControlState;
    private NavX mNavX;
    
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private Rotation2d mTargetHeading = new Rotation2d();
    private boolean mIsOnTarget = false;
    
    private int mInitLeftPosition;
    private int mInitRightPosition;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                // handleFaults();
                switch (mDriveControlState) {
                    case OPEN_LOOP:
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

            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };


    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = CANSpeedControllerFactory.createDefaultTalonFX(DriveConstants.kLeftDriveMasterId);
        TalonUtil.configureTalonFX(mLeftMaster, true, true);

        mLeftSlave = CANSpeedControllerFactory.createPermanentSlaveTalonFX(DriveConstants.kLeftDriveSlaveId, mLeftMaster);
        mLeftSlave.follow(mLeftMaster);
        TalonUtil.configureTalonFX(mLeftSlave, true, false);

        mRightMaster = CANSpeedControllerFactory.createDefaultTalonFX(DriveConstants.kRightDriveMasterId);
        TalonUtil.configureTalonFX(mRightMaster, false, true);

        mRightSlave = CANSpeedControllerFactory.createPermanentSlaveTalonFX(DriveConstants.kRightDriveSlaveId, mRightMaster);
        TalonUtil.configureTalonFX(mRightSlave, false, false);
        
        mRightSlave.follow(mRightMaster);
        
        mLeftSlave.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        mLeftSlave.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        mNavX = new NavX();
        mLeftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer.value, 10, 10);


        mLeftMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, .5));  
        mLeftSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, .5));  
        mRightMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, .5));  
        mRightSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, .5));  

        reloadGains();


        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mDriveControlState = DriveControlState.PATH_FOLLOWING;

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

        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mNavX.getFusedHeading()).rotateBy(mGyroOffset);
        mPeriodicIO.gyro_heading_deg = mNavX.getFusedHeading();

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / DriveConstants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.left_distance += deltaLeftTicks * (DriveConstants.kDriveWheelDiameterInches * DriveConstants.kDriveGearRatio);

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / DriveConstants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.right_distance += deltaRightTicks * (DriveConstants.kDriveWheelDiameterInches * DriveConstants.kDriveGearRatio);

    }

    @Override
    public synchronized void writePeriodicOutputs() {
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
        } else {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        }
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

    /**
     * Configures talons for velocity control for path following
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
            if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
                mDriveControlState = DriveControlState.PATH_FOLLOWING;
            }
        if(mTalonControlState != TalonControlState.VELOCITY){
            configureVelocityTalon();
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setPositionMagic(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
            
            mDriveControlState = DriveControlState.TURN_TO_HEADING;
        }
        if(mTalonControlState != TalonControlState.MOTION_MAGIC){
            configureMagicTalon();
        }
    
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }

    public synchronized void setPosition(DriveSignal signal) {
        if(mTalonControlState != TalonControlState.POSITION){
            configurePositionTalon();
        }
    
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }
    
    
    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            
            TalonUtil.setBrakeMode(mLeftMaster, shouldEnable);
            TalonUtil.setBrakeMode(mLeftSlave, shouldEnable);
            TalonUtil.setBrakeMode(mRightMaster, shouldEnable);
            TalonUtil.setBrakeMode(mRightSlave, shouldEnable);
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
        return ElementMath.rotationsToInches(getLeftEncoderRotations(), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
    }

    public double getRightEncoderDistance() {
        return ElementMath.rotationsToInches(getRightEncoderRotations(), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return ElementMath.rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DriveConstants.kDriveEncoderPPR, DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
    }
    
    public double getEncoderDinstance(){
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return ElementMath.rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DriveConstants.kDriveEncoderPPR, DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
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
         mInitLeftPosition = mPeriodicIO.left_position_ticks;
         mInitRightPosition = mPeriodicIO.right_position_ticks;
         setPositionMagic(new DriveSignal(mInitLeftPosition, mInitRightPosition));
 
 
             mTargetHeading = heading;
             mIsOnTarget = false;
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
            return;
        }

        DriveSignal wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));

        int ticksNeeded = (int)(ElementMath.inchesToRotations(wheel_delta.getRight(), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio) * DriveConstants.kDriveEncoderPPR ) ;

        int m_l_cur_pos = mPeriodicIO.left_position_ticks;
        int m_r_cur_pos = mPeriodicIO.right_position_ticks;

        int wantLeftPos = m_l_cur_pos - ticksNeeded;
        int wantRightPos = m_r_cur_pos + ticksNeeded;

        setPositionMagic(new DriveSignal(wantLeftPos, wantRightPos));
    }

    private void configureOpenTalon(){
        setBrakeMode(false);
    
        System.out.println("Switching to open loop");
    
        mLeftMaster.configOpenloopRamp(0.25);
        mRightMaster.configOpenloopRamp(0.25);

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


    public void reloadGains(){
        mLeftMaster.config_kP(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);
  
        mRightMaster.config_kP(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kHighGearVelocityControlSlot, AutoConstants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);
  

        //Position gains
        mLeftMaster.config_kP(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearPositionControlSlot, AutoConstants.kDrivePositionIZone, Constants.kLongCANTimeoutMs);
        mLeftMaster.configMotionCruiseVelocity((int)ElementMath.rpmToTicksPer100ms(ElementMath.inchesToRotations(AutoConstants.kDriveMaxAccel / 60, DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio), DriveConstants.kDriveEncoderPPR));
        mLeftMaster.configMotionAcceleration((int)ElementMath.rpmToTicksPer100ms(ElementMath.inchesToRotations(AutoConstants.kDriveMaxAccel / 60, DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio), DriveConstants.kDriveEncoderPPR));
    
  
        mRightMaster.config_kP(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearPositionControlSlot, AutoConstants.kDrivePositionKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearPositionControlSlot, AutoConstants.kDrivePositionIZone, Constants.kLongCANTimeoutMs);
        mRightMaster.configMotionCruiseVelocity((int)ElementMath.rpmToTicksPer100ms(ElementMath.inchesToRotations(AutoConstants.kDriveMaxAccel / 60, DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio), DriveConstants.kDriveEncoderPPR));
        mRightMaster.configMotionAcceleration((int)ElementMath.rpmToTicksPer100ms(ElementMath.inchesToRotations(AutoConstants.kDriveMaxAccel / 60, DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio), DriveConstants.kDriveEncoderPPR));
  
    }

    
    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public TalonFX getLeftMaster(){
        return mLeftMaster;
    }

    public TalonFX getLeftSlave(){
        return mLeftSlave;
    }

    public TalonFX getRightMaster(){
        return mRightMaster;
    }

    public TalonFX getRightSlave(){
        return mRightSlave;
    }

    @Override
    public void outputTelemetry() {
        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }


}

