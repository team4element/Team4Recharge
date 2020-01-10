package com.team4.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.CrashTracker;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.WheelHandlerConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


//TODO: add logic for color sensor, add motors
public class WheelHandler extends Subsystem{
    private static WheelHandler instance = null;

    private final ColorMatch mColorMatch = new ColorMatch();

    private ColorSensorV3 colorSensor;

    private PeriodicIO mPeriodicIO;

    private String mFMSSentString;

    private CurrentWheelMode mWheelMode;

    private boolean mIsReadyToControl;

    private double mSeenColorSetAmountOfTime = 0;

    private Color mPrevSeenColor = null;

    private boolean mFirstTime = true;

    private final Loop mLoop = new Loop(){
        public void onStart(double timestamp){

        }
        public void onLoop(double timestamp){
            switch(mWheelMode){
                case ROTATION:
                    if(mIsReadyToControl){
                        handleRotationControl();
                    }
                    break;
                case POSITION:
                    if(mIsReadyToControl){
                        handlePositionControl();
                    }     
                    break;
                default:
                    break;       
            }
        }
        public void onStop(double timestamp){
            stop();
        }
    };

    public static WheelHandler getInstance(){
        if(instance == null){
            instance = new WheelHandler();
        }
        return instance;
    }

    private WheelHandler(){
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        mPeriodicIO = new PeriodicIO();

        mColorMatch.addColorMatch(WheelHandlerConstants.kBlueTarget);
        mColorMatch.addColorMatch(WheelHandlerConstants.kGreenTarget);
        mColorMatch.addColorMatch(WheelHandlerConstants.kRedTarget);
        mColorMatch.addColorMatch(WheelHandlerConstants.kYellowTarget);  
        
        mFMSSentString = "Change this init String";

        mWheelMode = CurrentWheelMode.ROTATION;
    }

    @Override
    public void readPeriodicInputs() {
        try{
            mPeriodicIO.detected_color = colorSensor.getColor();        
            mPeriodicIO.ir = colorSensor.getIR();
            mPeriodicIO.proximity = colorSensor.getProximity();
            mPeriodicIO.matched_color = mColorMatch.matchClosestColor(mPeriodicIO.detected_color);

        }catch(Throwable t){
            CrashTracker.logThrowableCrash(t);
        }
    }

    @Override
    public void writePeriodicOutputs() {
    
    }

    public void handleRotationControl(){

    }
    public void handlePositionControl(){
        if(mFirstTime){
            resetColorDetection();
            mFirstTime = false;
        }
        try {
            
            if(mFMSSentString.length() > 0)
            {
                switch (mFMSSentString.charAt(0))
                {
              
                    case 'B' :
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mSeenColorSetAmountOfTime < 1){
                            //add rotate code to rotate until sees color
                        }else if(mSeenColorSetAmountOfTime > 1){
                            //this case should never be seen, report this error
                        }else if(mSeenColorSetAmountOfTime == 1){
                            //on works if on color initially, rotate one more time
                        }else{
                            //this case should never be possible, report if this condition is reached
                        }
                        break;
                    case 'G' :
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mSeenColorSetAmountOfTime < 1){
                            //add rotate code to rotate until sees color
                        }else if(mSeenColorSetAmountOfTime > 1){
                            //this case should never be seen, report this error
                        }else if(mSeenColorSetAmountOfTime == 1){
                            //on works if on color initially, rotate one more time
                        }else{
                            //this case should never be possible, report if this condition is reached
                        }
                        break;
                    case 'R' :
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mSeenColorSetAmountOfTime < 1){
                            //add rotate code to rotate until sees color
                        }else if(mSeenColorSetAmountOfTime > 1){
                            //this case should never be seen, report this error
                        }else if(mSeenColorSetAmountOfTime == 1){
                            //on works if on color initially, rotate one more time
                        }else{
                            //this case should never be possible, report if this condition is reached
                        }  
                        break;
                    case 'Y' :
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mPrevSeenColor != mPeriodicIO.matched_color.color){
                            if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                                mSeenColorSetAmountOfTime += 1;
                                mPrevSeenColor = mPeriodicIO.matched_color.color;
                            }
                        }
                        if(mSeenColorSetAmountOfTime < 1){
                            //add rotate code to rotate until sees color
                        }else if(mSeenColorSetAmountOfTime > 1){
                            //this case should never be seen, report this error
                        }else if(mSeenColorSetAmountOfTime == 1){
                            //on works if on color initially, rotate one more time
                        }else{
                            //this case should never be possible, report if this condition is reached
                        }
                        break;
                    default :
                        DriverStation.reportError("Sent data is corrupt", false);
                        break;
                }
            } else {
                    DriverStation.reportError("No data sent", false);
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
        }
    }

    @Override
    public void addLooper(ILooper mEnabledLooper) {
        mEnabledLooper.addLoop(mLoop);
    }

    public void setReadyToGo(boolean isReady){
        mIsReadyToControl = isReady;
    }

    public String getColorString(){
        if (mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget) {
            return "Blue";
          } else if (mPeriodicIO.matched_color.color == WheelHandlerConstants.kRedTarget) {
            return "Red";
          } else if (mPeriodicIO.matched_color.color == WheelHandlerConstants.kGreenTarget) {
            return "Green";
          } else if (mPeriodicIO.matched_color.color == WheelHandlerConstants.kYellowTarget) {
            return "Yellow";
          } else {
            return "Unknown";
          }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Detected Color", getColorString());
        SmartDashboard.putNumber("Red", mPeriodicIO.detected_color.red);
        SmartDashboard.putNumber("Green", mPeriodicIO.detected_color.green);
        SmartDashboard.putNumber("Blue", mPeriodicIO.detected_color.blue);
        SmartDashboard.putNumber("IR", mPeriodicIO.ir);
        SmartDashboard.putNumber("Proximity", mPeriodicIO.proximity);
    }
    
    @Override
    public boolean checkSystem() {
        return false;
    }
    
    @Override
    public void stop() {
        
    }

    public void updateFMSString(String gameData){
        mFMSSentString = gameData;
    }

    public void resetColorDetection(){
        mSeenColorSetAmountOfTime = 0;
    }

    public enum CurrentWheelMode{
        ROTATION,
        POSITION   
    }

    public static class PeriodicIO{
        public Color detected_color;
        public ColorMatchResult matched_color;
        public double ir;
        public int proximity;
    }

}