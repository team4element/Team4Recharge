package com.team4.robot;

import java.util.Arrays;

import com.ctre.phoenix.music.Orchestra;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.SynchronousPIDF;
import com.team4.lib.drivers.OrchestraUtil;
import com.team4.lib.util.DriveHelper;
import com.team4.lib.util.ElementMath;
import com.team4.lib.util.SongChooser;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.GamepadDriveControlBoard;
import com.team4.robot.controlboard.GamepadOperatorControlBoard;
import com.team4.robot.controlboard.IDriveControlBoard;
import com.team4.robot.controlboard.IOperatorControlBoard;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Intake;
import com.team4.robot.subsystems.Shooter;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.VisionTracker;
import com.team4.robot.subsystems.WheelHandler;
import com.team4.robot.subsystems.states.superstructure.SuperstructureState;

import edu.wpi.first.wpilibj.Compressor;
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
    private final Shooter mShooter = Shooter.getInstance();
    private final VisionTracker mVisionTracker = VisionTracker.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final WheelHandler mWheelHandler = WheelHandler.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final IDriveControlBoard mDriveControlBoard = GamepadDriveControlBoard.getInstance();
    private final IOperatorControlBoard mOperatorControlBoard = GamepadOperatorControlBoard.getInstance();

    private double kPeriod = .01; 
    private Notifier mNotifier;

    private Orchestra mOrchestra;

    private DriveHelper mDriveHelper = DriveHelper.getInstance();
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    private boolean compressVal = false;

    private Compressor mCompressor;

    private boolean intakeRelease = false;
    private boolean wasDrop = false;

    private boolean wheelRelease = false;

    private SynchronousPIDF autoSteerPID;

    private SongChooser mSongChooser;

    private HIDController(){
        mNotifier = new Notifier(runnable_);

        mOrchestra = new Orchestra(Arrays.asList(mDrive.getLeftMaster(), 
                                    mDrive.getLeftSlave(), 
                                    mDrive.getRightMaster(), 
                                    mDrive.getRightSlave(), 
                                    mShooter.getMasterTalon(), 
                                    mShooter.getSlaveTalon()
                                    ));
        
        OrchestraUtil.addOrchestra(mOrchestra);

        mCompressor = new Compressor(0);

        mSongChooser = new SongChooser();

        autoSteerPID = new SynchronousPIDF(AutoConstants.kLimelightAngleKp, AutoConstants.kLimelightAngleKi, AutoConstants.kLimelightAngleKd);

        autoSteerPID.setSetpoint(0);


    }

    private boolean running_ = false;
    private CrashTrackingRunnable runnable_ = new CrashTrackingRunnable(){
    
        @Override
        public void runCrashTracked() {
            if(running_){            
                double throttle = ElementMath.handleDeadband(mDriveControlBoard.getThrottle(), Constants.kJoystickThreshold);
                double turn;
                boolean wants_auto_steer = mDriveControlBoard.getVisionEnable() || mOperatorControlBoard.getVisionOverride();
                
                mVisionTracker.setVisionEnabled(wants_auto_steer);
                // mVisionTracker.setVisionEnabled(true);     
    
                if (mVisionTracker.isVisionEnabled() && mVisionTracker.isTargetFound()) {
                        turn = -autoSteerPID.calculate(VisionTracker.getInstance().getTargetHorizAngleDev());
                    // turn = 0d;
                    mDrive.setOpenLoop(mDriveHelper.elementDrive(throttle, turn, false));
                }else{
                    turn = -ElementMath.handleDeadband(-mDriveControlBoard.getTurn(), Constants.kJoystickThreshold);                
                    mDrive.setOpenLoop(mDriveHelper.elementDrive(throttle, turn, false));
                }

                if(mDriveControlBoard.getDropIntake()){
                    if(!intakeRelease){
                        if(!wasDrop){
                            mIntake.setDown();
                            wasDrop = true;
                            intakeRelease = true;
                        }else{
                            mIntake.setUp();
                            wasDrop = false;
                            intakeRelease = true;
                        }
                    }
                }else{
                    intakeRelease = false;
                }
                
                
                if(mDriveControlBoard.getWheelUp()){
                    if(!wheelRelease){
                        mWheelHandler.setReadyToGo();
                        wheelRelease = true;
                    }
                }else{
                    wheelRelease = false;
                }
                
                if(mDriveControlBoard.getWheelDown()){
                    if(!wheelRelease){
                        mWheelHandler.stopReady();
                        wheelRelease = true;
                    }
                }else{
                    wheelRelease = false;
                }

                if(mOperatorControlBoard.getShoot()){
                    mSuperstructure.setControlState(SuperstructureState.Convey_Shoot);
                }else if(mDriveControlBoard.getIntake()){
                    mSuperstructure.setControlState(SuperstructureState.Intake_Convey);
                }else if(mOperatorControlBoard.getBackConvey()){
                    mSuperstructure.setControlState(SuperstructureState.Convey_Convey);
                }else if(mDriveControlBoard.getRotationControl()){
                    mSuperstructure.setControlState(SuperstructureState.Enable_Wheel_Rotation);
                }else if(mDriveControlBoard.getPositionControl()){
                    mSuperstructure.setControlState(SuperstructureState.Enable_Wheel_Position);
                }else if(mOperatorControlBoard.getReverseSuperstructure()){
                    
                }else if(mDriveControlBoard.getClimbUp() || mOperatorControlBoard.getClimbUp()) {
                    
                }else if(mDriveControlBoard.getClimbDown() || mOperatorControlBoard.getClimbDown()) {
                    
                }else if(mOperatorControlBoard.getWinch() || mDriveControlBoard.getWinch()){
                
                }else{
                    mSuperstructure.setControlState(SuperstructureState.IDLE);
                }   
     


                }
            }
        };

    public void start(){
        running_ = true;
        OrchestraUtil.loadMusicSelection(false);
        mSuperstructure.resetCount();
        mNotifier.startPeriodic(kPeriod);
    }
    public void stop(){
        OrchestraUtil.stop();
        if(running_){   
           running_ = false;
        }
        instance = null;
    }
}