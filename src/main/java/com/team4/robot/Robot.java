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
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.CrashTracker;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeExecutor;
import com.team4.lib.loops.Looper;
import com.team4.lib.util.SubsystemManager;
import com.team4.lib.wpilib.TimedRobot;
import com.team4.robot.subsystems.Climber;
import com.team4.robot.subsystems.Conveyor;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Intake;
import com.team4.robot.subsystems.RobotStateEstimator;
import com.team4.robot.subsystems.Shooter;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.VisionTracker;
import com.team4.robot.subsystems.WheelHandler;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final Intake mIntake = Intake.getInstance();
    private final WheelHandler mWheelHandler = WheelHandler.getInstance();
    private final Climber mClimber = Climber.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    //Robot State Initialization
    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
  
    private final Compressor mCompressor = new Compressor(0);

    private AutoModeSelector mAutoSelector;
    private AutoModeExecutor mAutoModeExecutor;

  
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
                mIntake,
                mConveyor,
                mClimber,
                mWheelHandler,
                mSuperstructure);
              
                mSubsystemManager.configEnabledLoop(mEnabledLooper);
                mSubsystemManager.configDisabledLoop(mDisabledLooper);
        
                mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
                mDrive.setHeading(Rotation2d.identity());
        
                mAutoSelector = new AutoModeSelector();
                mAutoModeExecutor = new AutoModeExecutor();
                
                mCompressor.setClosedLoopControl(true);

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
      
            HIDController.getInstance().stop();

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
      
            mShooter.setBrakeMode(false);

            mAutoModeExecutor.start();
      
            mWheelHandler.updateFMSString(DriverStation.getInstance().getGameSpecificMessage());

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
        
        
        mDrive.setBrakeMode(false);
        mShooter.setBrakeMode(false);
        mRobotState.reset(Timer.getFPGATimestamp(), new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
        
        HIDController.getInstance().start();

      mWheelHandler.updateFMSString(DriverStation.getInstance().getGameSpecificMessage());

      mEnabledLooper.start();
    }

    @Override
    public void testInit() {
        try {


        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
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
  
                mShooter.setBrakeMode(false);
                mDrive.setBrakeMode(false);
        //   mDrive.zeroSensors();
          mAutoSelector.updateModeCreator();
            Optional<AutoModeBase> autoMode = mAutoSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
            }
            mAutoModeExecutor.setAutoMode(autoMode.get());
            mCompressor.stop();
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
            // mCompressor.start();
            // SmartDashboard.putBoolean("Current Data", mCompressor.getCompressorCurrentTooHighFault());
            // SmartDashboard.putBoolean("Pressure Switch", mCompressor.getPressureSwitchValue());
    }

    @Override
    public void testPeriodic() {
    }
     

}