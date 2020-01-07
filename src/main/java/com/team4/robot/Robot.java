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
import com.team254.lib.util.CrashTracker;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeExecutor;
import com.team4.lib.loops.Looper;
import com.team4.lib.util.SubsystemManager;
import com.team4.lib.wpilib.TimedRobot;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.RobotStateEstimator;
import com.team4.robot.subsystems.VisionTracker;

import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot{
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final VisionTracker mVisionTracker = VisionTracker.getInstance();

    private final Drive mDrive = Drive.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
  
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
                mVisionTracker);
              
                mSubsystemManager.configEnabledLoop(mEnabledLooper);
                mSubsystemManager.configDisabledLoops(mDisabledLooper);
        
                mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
                mDrive.setHeading(Rotation2d.identity());
        
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
      
          //   mLLManager.setAllLeds(Limelight.LedMode.OFF);
            // mLLManager.triggerOutputs();
      
            mDrive.setBrakeMode(false);
            // mLLManager.writePeriodicOutputs();
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
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());
      
            mAutoModeExecutor.start();
      
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
      mEnabledLooper.start();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mVisionTracker.readPeriodicInputs();
            mVisionTracker.writePeriodicOutputs();
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
  
          // mDrive.zeroSensors();]
          mAutoSelector.updateModeCreator();
            Optional<AutoModeBase> autoMode = mAutoSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
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
    }

    @Override
    public void testPeriodic() {
    }

}