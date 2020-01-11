/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team4.robot;

import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeExecutor;
import com.team4.lib.loops.Looper;
import com.team4.lib.util.DriveHelper;
import com.team4.lib.util.ElementMath;
import com.team4.lib.util.SubsystemManager;
import com.team4.lib.wpilib.TimedRobot;
import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.ControlBoard;
import com.team4.robot.controlboard.IControlBoard;
import com.team4.robot.subsystems.Conveyor;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.RobotStateEstimator;
import com.team4.robot.subsystems.Shooter;
import com.team4.robot.subsystems.Shooter.ShooterState;
import com.team4.robot.subsystems.VisionTracker;
import com.team4.robot.subsystems.WheelHandler;
import com.team4.robot.subsystems.Conveyor.ConveyorState;

import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot{
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    
    //Subsystem Initialization
    private final Drive mDrive = Drive.getInstance();
    private final VisionTracker mVisionTracker = VisionTracker.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Conveyor mConveyor = Conveyor.getInstance();
    private final WheelHandler mWheelHandler = WheelHandler.getInstance();

    //Robot State Initialization
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
  
    private AutoModeSelector mAutoSelector;
    private AutoModeExecutor mAutoModeExecutor;
  
    private IControlBoard mControlBoard = ControlBoard.getInstance();

    private DriveHelper mDriveHelper = DriveHelper.getInstance();
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    public Robot(){
        CrashTracker.logRobotConstruction();
    }
    
    @Override
    public void robotInit() {
        try {
            System.out.println("Starting robot init");
              mSubsystemManager.setSubsystems(
                mRobotStateEstimator,
                mDrive,
                mVisionTracker,
                mShooter,
                mConveyor,
                mWheelHandler);
              
                mSubsystemManager.configEnabledLoop(mEnabledLooper);
                mSubsystemManager.configDisabledLoops(mDisabledLooper);
        
                mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
                mDrive.setHeading(Rotation2d.identity());
        
                mAutoSelector = new AutoModeSelector();
                mAutoModeExecutor = new AutoModeExecutor();
                // mLLManager.setAllLeds(Limelight.LedMode.OFF);
        
                System.out.println("Finished robot init");
            } catch (Throwable t) {
              CrashTracker.logThrowableCrash(t);
            }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
      
            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeExecutor = new AutoModeExecutor();
      
      
            mDisabledLooper.start();
      

            // mDrive.setBrakeMode(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();
      
            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setHeading(Rotation2d.identity());
      
            mAutoModeExecutor.start();
      
            // mWheelHandler.updateFMSString(DriverStation.getInstance().getGameSpecificMessage());

            mEnabledLooper.start();
          } catch (Throwable t) {
              CrashTracker.logThrowableCrash(t);
              throw t;
          }
    }

    @Override
    public void teleopInit() {
      // Reset all auto mode state.
      
      if (mAutoModeExecutor != null) {
          mAutoModeExecutor.stop();
        }
        mAutoModeExecutor = new AutoModeExecutor();
    
        mDisabledLooper.stop();
        
    //   mWheelHandler.updateFMSString(DriverStation.getInstance().getGameSpecificMessage());

      mEnabledLooper.start();
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mWheelHandler.readPeriodicInputs();
            mWheelHandler.writePeriodicOutputs();
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
  
    @Override
    public void disabledPeriodic() {
        try {
              //   System.out.println("Zeroing Robot!");
                // mLLManager.triggerOutputs();
                // mLLManager.writePeriodicOutputs();
  
        //   mDrive.zeroSensors();
          mAutoSelector.updateModeCreator();
        //   Optional<AutoModeBase> autoMode = Optional.of(());
            Optional<AutoModeBase> autoMode = mAutoSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
            // mWheelHandler.updateFMSString(DriverStation.getInstance().getGameSpecificMessage());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }
  
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        // mShooter.handleDistanceRPM(100);
        manualControl();
    }

    @Override
    public void testPeriodic() {
    }

    public void manualControl(){
        double throttle = ElementMath.handleDeadband(mControlBoard.getThrottle(), Constants.kJoystickThreshold);
        double turn;
        boolean wants_auto_steer = mControlBoard.getVisionEnable();
 
        mVisionTracker.setVisionEnabled(wants_auto_steer);
    
        if (mVisionTracker.isVisionEnabled() && mVisionTracker.isTargetFound()) {
    
            turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.005, 0.1), -0.1);
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, true));
            // mDrive.autoSteer(Util.limit(throttle, 0.3), drive_aim_params.get());
        }else{
            turn = ElementMath.handleDeadband(-mControlBoard.getTurn(), Constants.kJoystickThreshold);
            mDrive.setOpenLoop(mDriveHelper.elementDrive(throttle, turn, false));
            
        }


        if(mControlBoard.getShoot()){
            mShooter.setOpenLoop(.8);
        }else{
            mShooter.setOpenLoop(0); 
        }

        if(mControlBoard.getMoveConveyor()){
            mConveyor.setControlState(ConveyorState.FORWARD);
        }else{
            mConveyor.setControlState(ConveyorState.IDLE);
        }


    }

}