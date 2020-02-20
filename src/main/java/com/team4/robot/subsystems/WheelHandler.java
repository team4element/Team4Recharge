package com.team4.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.team254.lib.util.CrashTracker;
import com.team4.lib.drivers.CANSpeedControllerFactory;
import com.team4.lib.drivers.TalonUtil;
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

    private ColorSensorV3 mColorSensor;

    private PeriodicIO mPeriodicIO;

    private String mFMSSentString;

    private CurrentWheelMode mWheelMode;

    private boolean mIsReadyToControl;

    private double mSeenColorSetAmountOfTime = 0;

    private Color mPrevSeenColor = null;
    private Color mFirstSeenColor = null;

    private boolean mFirstTime = true;

    private TalonSRX mMotor;

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
                case IDLE:
                    mPeriodicIO.demand = 0;
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
        mPeriodicIO = new PeriodicIO();
        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        mMotor = CANSpeedControllerFactory.createDefaultTalonSRX(WheelHandlerConstants.kWheelHandlerMotorID);
        mMotor.configNeutralDeadband(0.04, 0);
        TalonUtil.setBrakeMode(mMotor, true);

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
            mPeriodicIO.detected_color = mColorSensor.getColor();        
            mPeriodicIO.ir = mColorSensor.getIR();
            mPeriodicIO.proximity = mColorSensor.getProximity();
            mPeriodicIO.matched_color = mColorMatch.matchClosestColor(mPeriodicIO.detected_color);

        }catch(Throwable t){
            CrashTracker.logThrowableCrash(t);
        }
    }

    @Override
    public void writePeriodicOutputs() {
    
    }

    public void handleRotationControl(){
        if(mFirstTime){
            resetColorDetection();
            logFirstColor();
            mFirstTime = false;
        }

        if(mPrevSeenColor != mFirstSeenColor && mPeriodicIO.matched_color.color == mFirstSeenColor){
            mSeenColorSetAmountOfTime += 1;
        }
        
        if(mSeenColorSetAmountOfTime >= 7){
            mPeriodicIO.demand = 0.0;
            mWheelMode = CurrentWheelMode.IDLE;
        }else{
            mPeriodicIO.demand = 0.5;
        }
        
        mPrevSeenColor = mPeriodicIO.matched_color.color;
    }

    public void logFirstColor(){
        mFirstSeenColor = mPeriodicIO.matched_color.color;
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
                        if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kGreenTarget){
                            mPeriodicIO.demand = -.4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kRedTarget){
                            mPeriodicIO.demand = .7;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kYellowTarget){
                            mPeriodicIO.demand = .4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                            mPeriodicIO.demand = 0;
                        }       
                        break;
                    case 'G' :
                        if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kRedTarget){
                            mPeriodicIO.demand = -.4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kYellowTarget){
                            mPeriodicIO.demand = .7;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                            mPeriodicIO.demand = .4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kGreenTarget){
                            mPeriodicIO.demand = 0;
                        }
                        break;
                    case 'R' :
                        if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kYellowTarget){
                            mPeriodicIO.demand = -.4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                            mPeriodicIO.demand = .7;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kGreenTarget){
                            mPeriodicIO.demand = .4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kRedTarget){
                            mPeriodicIO.demand = 0;
                        }

                        break;
                    case 'Y' :
                        if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kBlueTarget){
                            mPeriodicIO.demand = -.4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kGreenTarget){
                            mPeriodicIO.demand = .7;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kRedTarget){
                            mPeriodicIO.demand = .4;
                        }else if(mPeriodicIO.matched_color.color == WheelHandlerConstants.kYellowTarget){
                            mPeriodicIO.demand = 0;
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

    public void setWheelControlState(CurrentWheelMode mode){
        if(mWheelMode != mode){
            mWheelMode = mode;
        }
    }

    public enum CurrentWheelMode{
        ROTATION,
        POSITION,   
        IDLE
    }

    protected static class PeriodicIO{
        public Color detected_color;
        public ColorMatchResult matched_color;
        public double ir;
        public int proximity;

        public double demand;
    }

}