package com.team4.robot;

import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.SynchronousPIDF;
import com.team4.lib.util.DriveHelper;
import com.team4.lib.util.ElementMath;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.ControlBoard;
import com.team4.robot.controlboard.IControlBoard;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.VisionTracker;
import com.team4.robot.subsystems.states.superstructure.SuperstructureState;

import edu.wpi.first.wpilibj.Notifier;

public class HIDController{
    private static HIDController instance = null;

    public static HIDController getInstance(){
        if(instance == null){
            instance = new HIDController();
        }
        return instance;
    }

    private final Drive mDrive = Drive.getInstance();
    private final VisionTracker mVisionTracker = VisionTracker.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private double kPeriod = .01; 
    private Notifier mNotifier;

    private IControlBoard mControlBoard = ControlBoard.getInstance();
    private DriveHelper mDriveHelper = DriveHelper.getInstance();
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();


    private SynchronousPIDF autoSteerPID;


    private HIDController(){
        mNotifier = new Notifier(runnable_);

        
        autoSteerPID = new SynchronousPIDF(AutoConstants.kLimelightAngleKp, AutoConstants.kLimelightAngleKi, AutoConstants.kLimelightAngleKd);

        autoSteerPID.setSetpoint(0);


    }

    private boolean running_ = false;
    private CrashTrackingRunnable runnable_ = new CrashTrackingRunnable(){
    
        @Override
        public void runCrashTracked() {
            if(running_){            
                double throttle = ElementMath.handleDeadband(mControlBoard.getThrottle(), Constants.kJoystickThreshold);
                double turn;
                boolean wants_auto_steer = mControlBoard.getVisionEnable();
                
                mVisionTracker.setVisionEnabled(wants_auto_steer);
                // mVisionTracker.setVisionEnabled(true);     
    
                if (mVisionTracker.isVisionEnabled() && mVisionTracker.isTargetFound()) {
                        turn = autoSteerPID.calculate(VisionTracker.getInstance().getTargetHorizAngleDev());
                    // turn = 0d;
                    // turn = Math.max(Math.min(VisionTracker.getInstance().getTargetHorizAngleDev() * 0.01, 0.1), -0.1);
                    mDrive.setOpenLoop(mDriveHelper.elementDrive(throttle, turn, mControlBoard.getQuickTurn()));
                }else{
                    turn = ElementMath.handleDeadband(-mControlBoard.getTurn(), Constants.kJoystickThreshold);                
                    mDrive.setOpenLoop(mDriveHelper.elementDrive(throttle, turn, mControlBoard.getQuickTurn()));
                }
                
            
                if(mControlBoard.getShoot()){
                    mSuperstructure.setControlState(SuperstructureState.Convey_Shoot);
                }else if(mControlBoard.getIntake()){
                    mSuperstructure.setControlState(SuperstructureState.Intake_Convey);
                }else{
                    mSuperstructure.setControlState(SuperstructureState.IDLE);
                }
            }   
        }
    };

    public void start(){
        running_ = true;
        mNotifier.startPeriodic(kPeriod);
        mSuperstructure.resetCount();
    }
    public void stop(){
        if(running_){   
           running_ = false;
        }
        instance = null;
    }
}