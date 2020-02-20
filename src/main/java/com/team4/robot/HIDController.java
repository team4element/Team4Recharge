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
import com.team4.robot.controlboard.ControlBoard;
import com.team4.robot.controlboard.IControlBoard;
import com.team4.robot.subsystems.Drive;
import com.team4.robot.subsystems.Shooter;
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
    private final Shooter mShooter = Shooter.getInstance();
    private final VisionTracker mVisionTracker = VisionTracker.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private double kPeriod = .01; 
    private Notifier mNotifier;

    private Orchestra mOrchestra;

    private IControlBoard mControlBoard = ControlBoard.getInstance();
    private DriveHelper mDriveHelper = DriveHelper.getInstance();
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();

    private boolean initSong = false;
    private boolean justPause = false;
    private boolean songJustChange = false;

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

        mSongChooser = new SongChooser();

        autoSteerPID = new SynchronousPIDF(AutoConstants.kLimelightAngleKp, AutoConstants.kLimelightAngleKi, AutoConstants.kLimelightAngleKd);

        autoSteerPID.setSetpoint(0);


    }

    private boolean running_ = false;
    private CrashTrackingRunnable runnable_ = new CrashTrackingRunnable(){
    
        @Override
        public void runCrashTracked() {
            if(running_){            
                mSongChooser.updateSelectedChoice();

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

                

                if(mControlBoard.getPauseSong()){
                    if(!initSong){
                        OrchestraUtil.playLoadedSong();
                        initSong = true;
                    }else{
                        if(!justPause){
                            OrchestraUtil.pause();
                            justPause = true;
                        }
                    }
                }else if(mControlBoard.getStopSong()){
                    if(!initSong){
                        OrchestraUtil.playLoadedSong();
                        initSong = true;
                    }else{
                        if(!justPause){
                            OrchestraUtil.stop();
                            justPause = true;
                        }
                    }
                }else{
                    justPause = false;
                }

                if(mControlBoard.getNextSong()){
                    if(!songJustChange){
                        OrchestraUtil.loadMusicSelection(true);
                        songJustChange = true;
                    }
                }else if(mControlBoard.getPrevSong()){
                    if(!songJustChange){
                        OrchestraUtil.loadMusicSelection(false);
                        songJustChange = true;
                    }
                }else{
                    songJustChange = false;
                }

                OrchestraUtil.playSong(mSongChooser.getSelectedSong());

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
        if(running_){   
           running_ = false;
        }
        instance = null;
    }
}