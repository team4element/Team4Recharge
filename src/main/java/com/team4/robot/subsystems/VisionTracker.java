package com.team4.robot.subsystems;

import java.util.ArrayList;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.MovingAverage;
import com.team4.lib.loops.ILooper;
import com.team4.lib.loops.Loop;
import com.team4.lib.util.Subsystem;
import com.team4.robot.constants.TargetingConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTracker extends Subsystem {
	private static VisionTracker mInstance = new VisionTracker();
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private boolean mVisionEnabled = false;

	private NetworkTable mCurrentTargetingLimelightNT;
	private NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");

	private double mLEDMode = 0;

	public static VisionTracker getInstance() {
		return mInstance;
	}

	private VisionTracker() {
		mCurrentTargetingLimelightNT = limelightNT;
	}
	
	private final Loop mLoop = new Loop() {
		
			@Override
		public void onStart(double timestamp) {
			synchronized (VisionTracker.this) {
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (VisionTracker.this) {
						mPeriodicIO.pipelineFront = mVisionEnabled ? 0 : 1;
						mCurrentTargetingLimelightNT = limelightNT;
			}
		}

		@Override
		public void onStop(double timestamp) {
			stop();
		}
	};


	@Override
	public void addLooper(ILooper mEnabledLooper) {
		mEnabledLooper.addLoop(mLoop);
	}

	public void stop() {
		setVisionEnabled(false);
	}
	

	public boolean isVisionEnabled() {
		return mVisionEnabled;
	}

	public boolean isTargetFound() {
		return mVisionEnabled && mPeriodicIO.targetValid > 0;
	}

	public boolean isTargetAreaReached() { return mPeriodicIO.targetArea >= .04; }

	public double getTargetDistance() {
		return mVisionEnabled ? mPeriodicIO.targetDistance : 0;
	}

	public double getTargetHorizAngleDev() {
		return mVisionEnabled ? mCurrentTargetingLimelightNT.getEntry("tx").getDouble(0) : 0;
	}

	public double getTargetVertAngleDev() {
		return mVisionEnabled ? mPeriodicIO.targetVerticalDeviation : 0;
	}

	public double getLensHeight(){
		return TargetingConstants.kFloorToLens;
	}

	public synchronized double getSkewFactor() {
		return mPeriodicIO.calculatedSkewFactor.getAverage();
	}

	public synchronized void setVisionEnabled(boolean enabled) {
		mVisionEnabled = enabled;
		SmartDashboard.putBoolean("Vision Enabled", mVisionEnabled);
	}


	public double getTargetSkew() {
		return mPeriodicIO.targetSkew;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		try {
			if (mVisionEnabled) {
				mPeriodicIO.targetValid = mCurrentTargetingLimelightNT.getEntry("tv").getDouble(0);
				mPeriodicIO.targetHorizontalDeviation = mCurrentTargetingLimelightNT.getEntry("tx").getDouble(0);
				mPeriodicIO.targetVerticalDeviation = mCurrentTargetingLimelightNT.getEntry("ty").getDouble(0);
				mPeriodicIO.targetArea = mCurrentTargetingLimelightNT.getEntry("ta").getDouble(0);
				mPeriodicIO.targetSkew = mCurrentTargetingLimelightNT.getEntry("ts").getDouble(0);
				mPeriodicIO.targetLatency = mCurrentTargetingLimelightNT.getEntry("tl").getDouble(0);
				mPeriodicIO.getPipelineValue = mCurrentTargetingLimelightNT.getEntry("getpipe").getDouble(0);
				mPeriodicIO.cameraA1 = Math.toDegrees(Math.atan((TargetingConstants.kFloorToTarget - TargetingConstants.kFloorToLens)/(144) /*240 inches to find what A1 is */))-mPeriodicIO.targetVerticalDeviation;
	
				mPeriodicIO.targetDistance =
				(TargetingConstants.kFloorToTarget - TargetingConstants.kFloorToLens) /
				/*	Math.toDegrees(*/Math.tan(Math.toRadians(TargetingConstants.kFloorToLensAngle + mPeriodicIO.targetVerticalDeviation))/*)*/;
				// mPeriodicIO.targetDistance += 20;
		
				try {
					
					double xArr[] = mCurrentTargetingLimelightNT.getEntry("tcornx").getDoubleArray(new double[]{0});
					double yArr[] = mCurrentTargetingLimelightNT.getEntry("tcorny").getDoubleArray(new double[]{0});

					if (xArr.length == yArr.length && xArr.length > 4) {
						mPeriodicIO.pointArray.clear();
						for (int i = 0; i < xArr.length; i++) {
							mPeriodicIO.pointArray.add(new Translation2d(xArr[i], yArr[i]));
						}

						Translation2d upperLeftPoint = mPeriodicIO.pointArray.get(0);
						Translation2d lowerLeftPoint = mPeriodicIO.pointArray.get(1);
						Translation2d lowerRightPoint = mPeriodicIO.pointArray.get(yArr.length - 2);
						Translation2d upperRightPoint = mPeriodicIO.pointArray.get(yArr.length - 1);

						double upperLineSlope = Math.abs((upperRightPoint.y() - upperLeftPoint.y()) / (upperRightPoint.x() - upperLeftPoint.x()));
						double lowerLineSlope = (lowerRightPoint.y() - lowerLeftPoint.y()) / (lowerRightPoint.x() - lowerLeftPoint.x());
						mPeriodicIO.calculatedSkewFactor.add(Math.toDegrees(Math.atan((upperLineSlope + Math.abs(lowerLineSlope)) / 2.0)) * Math.signum(lowerLineSlope));
						

						
					} else {
						mPeriodicIO.calculatedSkewFactor.clear();
					}
				} catch (Exception ex) {
					CrashTracker.logThrowableCrash(ex);
				}
				
				// linearDistance = (1.1751* mPeriodicIO.targetDistance) - 13;
				// exponDistance = 71.737 * Math.pow(Math.E, (.0056*mPeriodicIO.targetDistance));
				// powDistance = .77 * Math.pow(mPeriodicIO.targetDistance, 1.0688);
				// quadDistance = (-.0007 * Math.pow(mPeriodicIO.targetDistance, 2)) + (1.4528 * mPeriodicIO.targetDistance) - 39.362 ; 
		}
			else {
				mPeriodicIO.targetValid = 0;
				mPeriodicIO.targetHorizontalDeviation = 0;
				mPeriodicIO.targetVerticalDeviation = 0;
				mPeriodicIO.targetArea = 0;
				mPeriodicIO.targetSkew = 0;
				mPeriodicIO.targetLatency = 0;
				mPeriodicIO.getPipelineValue = 0;
				mPeriodicIO.targetDistance = 0;
				mPeriodicIO.cameraA1 = 0;
				mPeriodicIO.calculatedSkewFactor.clear();
			}
		}
		catch (Exception ex) {
			CrashTracker.logThrowableCrash(ex);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		try {
			limelightNT.getEntry("pipeline").setNumber(mPeriodicIO.pipelineFront);
			mCurrentTargetingLimelightNT.getEntry("ledMode").setNumber(mLEDMode);
		}
		catch (Exception ex) {
			CrashTracker.logThrowableCrash(ex);
		}

//		NetworkTableEntry ledMode = mCurrentTargetingLimelightNT.getValue().getEntry("ledMode");
//		NetworkTableEntry camMode = mCurrentTargetingLimelightNT.getValue().getEntry("camMode");
//		NetworkTsableEntry stream = mCurrentTargetingLimelightNT.getValue().getEntry("stream");
//		NetworkTableEntry snapshot = mCurrentTargetingLimelightNT.getValue().getEntry("snapshot");
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber("Distance", mPeriodicIO.targetDistance);
		SmartDashboard.putNumber("Camera A1", mPeriodicIO.cameraA1);
	}

	public void setLEDMode(LedMode mode){
		switch(mode){
			case PIPELINE:
				mLEDMode = mCurrentTargetingLimelightNT.getEntry("LedMode").getDouble(0.0);
			case ON:
				mLEDMode = 0;
				break;
			case OFF:
				mLEDMode = 2;
			case BLINK:
				mLEDMode = 1;
		}
	}

	protected static class PeriodicIO {
		//Making members public here will automatically add them to logs
		//Read values
		public double targetValid;
		public double targetHorizontalDeviation;
		public double targetVerticalDeviation;
		public double targetArea;
		public double targetSkew;
		public double targetLatency;
		public double targetDistance;
		public double cameraA1;
		public double getPipelineValue;
		public MovingAverage calculatedSkewFactor = new MovingAverage(10);

		public ArrayList<Translation2d> pointArray = new ArrayList<>();

		//Written values
		public int pipelineFront;
		public int pipelineBack;
	}

	public enum LedMode{
		PIPELINE,
		ON,
		BLINK,
		OFF
	}
}
