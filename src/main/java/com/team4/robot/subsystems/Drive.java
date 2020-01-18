package com.team4.robot.subsystems;

import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.drivers.MotorChecker;
import com.team4.lib.drivers.NavX;
import com.team4.lib.drivers.TalonSRXChecker;
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
import com.team4.robot.planners.DriveMotionPlanner;
import com.team4.robot.subsystems.states.DriveControlState;
import com.team4.robot.subsystems.states.TalonControlState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    
    private static Drive mInstance = new Drive();
    
    private ReentrantLock mSubsystemLock = new ReentrantLock();
    
    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster;
    private final VictorSPX mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;
    // Control states
    private DriveControlState mDriveControlState;
    private TalonControlState mTalonControlState;
    private NavX mNavX;
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private boolean mIsOnTarget;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private boolean mOverrideTrajectory = false;
    private Rotation2d mTargetHeading = new Rotation2d();

    private int mInitLeftPosition;
    private int mInitRightPosition;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
                startLogging();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        /**
                         * TODO: test if it works
                         *
                         * if(mMotionPlanner.hasTrajectory())
                         * {
                            updatePathFollower();
                         * }
                         */
                        updatePathFollower();
                        break;
                    case TURN_TO_HEADING:
                        updateTurnToHeading();
                        break;
                    case DRIVE_VELOCITY:
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState.toString());
                        break;
                }
                
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            stopLogging();
        }
    };


    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = CANSpeedControllerFactory.createDefaultTalonSRX(DriveConstants.kLeftDriveMasterId);
        TalonUtil.configureTalonSRX(mLeftMaster, true);

        mLeftSlaveA = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kLeftDriveSlaveAId, mLeftMaster);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kLeftDriveSlaveBId, mLeftMaster);
        mLeftSlaveB.setInverted(false);

        mRightMaster = CANSpeedControllerFactory.createDefaultTalonSRX(DriveConstants.kRightDriveMasterId);
        TalonUtil.configureTalonSRX(mRightMaster, false);

        mRightSlaveA = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kRightDriveSlaveAId, mRightMaster);
        mRightSlaveA.setInverted(true);

        mRightSlaveB = CANSpeedControllerFactory.createPermanentSlaveVictor(DriveConstants.kRightDriveSlaveBId, mRightMaster);
        mRightSlaveB.setInverted(true);


        reloadGains();

        mNavX = new NavX();

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public static Drive getInstance() {
        return mInstance;
    }


    @Override
    public void addLooper(ILooper in) {
        in.addLoop(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            System.out.println("Switching to open loop");
            System.out.println(signal);
            setDriveControlMode(DriveControlState.OPEN_LOOP);
            // mLeftMaster.configNeutralDeadband(0.04, 0);
            // mRightMaster.configNeutralDeadband(0.04, 0);
        }
        if(mTalonControlState != TalonControlState.OPEN){
            configureOpenTalon();
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            setDriveControlMode(DriveControlState.PATH_FOLLOWING);
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
        setDriveControlMode(DriveControlState.TURN_TO_HEADING);
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

    /**
   * Configures talons for velocity control
   */
  public synchronized void setVelocityInchesPerSecond(DriveSignal signal) {
    if (!(mDriveControlState == DriveControlState.DRIVE_VELOCITY)) {
        if(mTalonControlState != TalonControlState.VELOCITY){
            configureVelocityTalon();
        }
    }
    if (mDriveControlState != DriveControlState.DRIVE_VELOCITY){
        setDriveControlMode(DriveControlState.DRIVE_VELOCITY);
    }

    // SmartDashboard.putNumber("LeftIPS", signal.getLeft());
    // SmartDashboard.putNumber("RightIPS",signal.getRight());
    double left_velocity_rev = ElementMath.inchesToRotations((signal.getLeft())*(2*Math.PI), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
    double right_velocity_rev = ElementMath.inchesToRotations((signal.getRight())*(2*Math.PI), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);

    mPeriodicIO.left_demand = ElementMath.radiansPerSecondToTicksPer100ms(left_velocity_rev, DriveConstants.kDriveEncoderPPR);
    mPeriodicIO.right_demand =  ElementMath.radiansPerSecondToTicksPer100ms(right_velocity_rev, DriveConstants.kDriveEncoderPPR);
    mPeriodicIO.left_feedforward = left_velocity_rev * DriveConstants.kDriveKv;
    mPeriodicIO.right_feedforward = right_velocity_rev * DriveConstants.kDriveKv;
}

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            setBrakeMode(true);
            setDriveControlMode(DriveControlState.PATH_FOLLOWING);
        }
    }

    /**
     * Configures the drivebase to turn to a desired heading
     */
    public synchronized void setWantTurnToHeading(Rotation2d heading) {
        setDriveControlMode(DriveControlState.TURN_TO_HEADING);
         // updatePositionSetpoint(getLeftEncoderDistance(), getRightEncoderDistance());
         mInitLeftPosition = mPeriodicIO.left_position_ticks;
         mInitRightPosition = mPeriodicIO.right_position_ticks;
         // setPositionMagic(new DriveSignal(mInitLeftPosition, mInitRightPosition));
         setPosition(new DriveSignal(mInitLeftPosition, mInitRightPosition));
 
 
         if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
             mTargetHeading = heading;
             mIsOnTarget = false;
 
             final Rotation2d field_to_robot = getHeading();
             final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);
 
             DriveSignal wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
            setPositionMagic(wheel_delta);    
        }
         
     }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING) {
            return mIsOnTarget;
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        TalonUtil.setBrakeMode(mLeftMaster, on);
        TalonUtil.setBrakeMode(mRightMaster, on);

        
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }

    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mNavX.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
        SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
        SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
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
        return ElementMath.rotationsToInches((getRightVelocityNativeUnits() * 10.0 / DriveConstants.kDriveEncoderPPR), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return ElementMath.rotationsToInches((getLeftVelocityNativeUnits() * 10.0 / DriveConstants.kDriveEncoderPPR), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / DriveConstants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(ElementMath.radiansPerSecondToTicksPer100ms(output.left_velocity, DriveConstants.kDriveEncoderPPR), ElementMath.radiansPerSecondToTicksPer100ms(output.right_velocity, DriveConstants.kDriveEncoderPPR)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = ElementMath.radiansPerSecondToTicksPer100ms(output.left_accel, DriveConstants.kDriveEncoderPPR) / 1000.0;
                mPeriodicIO.right_accel = ElementMath.radiansPerSecondToTicksPer100ms(output.right_accel, DriveConstants.kDriveEncoderPPR) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading() {
        final Rotation2d field_to_robot = getHeading();
        // System.out.println("Running updateTurnToHeading");
        // final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
        
        // System.out.println("FIELD TO VEHICLE IS  " + field_to_robot.getDegrees());

        // Figure out the rotation necessary to turn to face the goal.


        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);
        

        // Check if we are on target
        final double kGoalPosTolerance = 2.0; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance && Math.abs(getLeftLinearVelocity()) < kGoalVelTolerance && Math.abs(getRightLinearVelocity()) < kGoalVelTolerance) {
            // if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance) {

            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            setPosition(new DriveSignal(mPeriodicIO.left_position_ticks, mPeriodicIO.right_position_ticks));

            return;
        }

        DriveSignal wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        int ticksNeeded = (int)(ElementMath.inchesToRotations((wheel_delta.getRight() * DriveConstants.kDriveEncoderPPR), DriveConstants.kDriveWheelCircumferenceInches, DriveConstants.kDriveGearRatio));

        // System.out.println("ticksNeeded: " + ticksNeeded);

        int m_l_cur_pos = mPeriodicIO.left_position_ticks;
        int m_r_cur_pos = mPeriodicIO.right_position_ticks;

        int wantLeftPos = m_l_cur_pos - ticksNeeded;
        int wantRightPos = m_r_cur_pos + ticksNeeded;


        setPositionMagic(new DriveSignal(wantLeftPos, wantRightPos));
        // setPosition(new DriveSignal(wantLeftPos, wantRightPos));
    }


    public synchronized void reloadGains() {
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
    
        mLeftMaster.config_kP(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearVelocityControlSlot, AutoConstants.kDrivePositionIZone, Constants.kLongCANTimeoutMs);
    
        mLeftMaster.configMotionCruiseVelocity((int)ElementMath.radiansPerSecondToTicksPer100ms((AutoConstants.kDriveMaxVelocity*(2*Math.PI)/60), DriveConstants.kDriveEncoderPPR));
        mLeftMaster.configMotionAcceleration((int)ElementMath.radiansPerSecondToTicksPer100ms((AutoConstants.kDriveMaxAccel*(2*Math.PI)/60), DriveConstants.kDriveEncoderPPR));

        mRightMaster.configMotionCruiseVelocity((int)ElementMath.radiansPerSecondToTicksPer100ms((AutoConstants.kDriveMaxVelocity*(2*Math.PI)/60), DriveConstants.kDriveEncoderPPR));
        mRightMaster.configMotionAcceleration((int)ElementMath.radiansPerSecondToTicksPer100ms((AutoConstants.kDriveMaxAccel*(2*Math.PI)/60), DriveConstants.kDriveEncoderPPR));
    }

    @Override
    public void writeToLog() {
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mNavX.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            mPeriodicIO.left_distance += deltaLeftTicks * DriveConstants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.left_distance += deltaLeftTicks * DriveConstants.kDriveWheelDiameterInches;
        }

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            mPeriodicIO.right_distance += deltaRightTicks * DriveConstants.kDriveWheelDiameterInches;
        } else {
            mPeriodicIO.right_distance += deltaRightTicks * DriveConstants.kDriveWheelDiameterInches;
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        
    // System.out.println("Left Demand: " + mPeriodicIO.left_demand + "Left Arbitrary: " + mPeriodicIO.left_feedforward);
    //   if (mDriveControlState == DriveControlState.OPEN_LOOP) {
        if (mTalonControlState == TalonControlState.OPEN) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand*.65);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            // mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            // mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        }else if(mTalonControlState == TalonControlState.MOTION_MAGIC){
      //         if(mDriveControlState == DriveControlState.TURN_TO_HEADING){
            mLeftMaster.set(ControlMode.MotionMagic, (int)mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.MotionMagic, (int)mPeriodicIO.right_demand);
        }else if(mTalonControlState == TalonControlState.VELOCITY){
              //   else { //default case velocity
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + AutoConstants.kDriveVelocityKd * mPeriodicIO.left_accel / (DriveConstants.kDriveEncoderPPR/4.0));
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + AutoConstants.kDriveVelocityKd * mPeriodicIO.right_accel / (DriveConstants.kDriveEncoderPPR/4.0));
        }else if(mTalonControlState == TalonControlState.POSITION){
            mLeftMaster.set(ControlMode.Position, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.Position, mPeriodicIO.right_demand);
        }else{ // Default Case Velocity
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.left_feedforward + AutoConstants.kDriveVelocityKd * mPeriodicIO.left_accel / (DriveConstants.kDriveEncoderPPR/4.0));
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.right_feedforward + AutoConstants.kDriveVelocityKd * mPeriodicIO.right_accel / (DriveConstants.kDriveEncoderPPR/4.0));
        }
    }

    @Override
    public boolean checkSystem() {
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

    
  private void configureOpenTalon(){
    setBrakeMode(false);

    System.out.println("Switching to open loop");
    
    setTalonControlMode(TalonControlState.OPEN);
  }

  private void configureVelocityTalon(){

    setBrakeMode(true);
    mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
    mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);

    mLeftMaster.configClosedloopRamp(0);
    mRightMaster.configClosedloopRamp(0);

    setTalonControlMode(TalonControlState.VELOCITY);

    System.out.println("Switching to velocity");

  }

  private void configureMagicTalon(){

    setBrakeMode(true);
    mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
    mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
    
    setTalonControlMode(TalonControlState.MOTION_MAGIC);

    System.out.println("Switching to Magic");
  }

  private void configurePositionTalon(){

    setBrakeMode(true);
    mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
    mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
    
    setTalonControlMode(TalonControlState.POSITION);

    System.out.println("Switching to Position");
  }

    private void setDriveControlMode(DriveControlState desiredState){
        if(mDriveControlState != desiredState){
            try {
                mSubsystemLock.lock();
                    System.out.println("Updating Drive Control Mode to " + desiredState.name());
                    mDriveControlState = desiredState;
                    System.out.println("Update Successful");
                mSubsystemLock.unlock();
            } catch (Throwable t) {
                CrashTracker.logThrowableCrash(t);
            }
        }
    }

    private void setTalonControlMode(TalonControlState desiredState){
        if(mTalonControlState != desiredState){
            try {
                mSubsystemLock.lock();
                    System.out.println("Updating Talon Control Mode to " + desiredState.name());
                    mTalonControlState = desiredState;
                    System.out.println("Update Successful");
                mSubsystemLock.unlock();
            } catch (Throwable t) {
                CrashTracker.logThrowableCrash(t);
            }
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    protected static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}

