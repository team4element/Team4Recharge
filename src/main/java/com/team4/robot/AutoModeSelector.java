package com.team4.robot;

import java.util.Optional;

import com.team4.lib.autobase.AutoModeBase;
import com.team4.robot.auto.modes.DoNothingMode;
import com.team4.robot.auto.modes.LeftToRendAndShootMode;
import com.team4.robot.auto.modes.LeftToTrenchAndShootMode;
import com.team4.robot.auto.modes.MidToRendAndShootMode;
import com.team4.robot.auto.modes.MidToTrenchAndShootMode;
import com.team4.robot.auto.modes.RightToRendAndShootMode;
import com.team4.robot.auto.modes.RightToTrenchAndShootMode;
import com.team4.robot.auto.modes.SteerAndShootAction;
import com.team4.robot.auto.modes.TestMode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {

    //TODO: add automodes and starting positions

    public enum StartingPosition {
        LEFT,
        RIGHT,
        MID
    }

    enum DesiredMode {
        DO_NOTHING,
        TEST,
        RENDEZVOUS,
        TRENCH,
        SHOOT
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private final SendableChooser<DesiredMode> mModeChooser;
    private final SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Start Position - Middle", StartingPosition.MID);
        mStartPositionChooser.addOption("Start Position - Right", StartingPosition.RIGHT);
        mStartPositionChooser.addOption("Start Position - Left", StartingPosition.LEFT);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Rendezvous", DesiredMode.RENDEZVOUS);
        mModeChooser.addOption("Trench", DesiredMode.TRENCH);
        mModeChooser.addOption("Test", DesiredMode.TEST);
        mModeChooser.addOption("Shoot Mode", DesiredMode.SHOOT);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        final DesiredMode desiredMode = mModeChooser.getSelected();
        final StartingPosition staringPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition) {
            // System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    // + ", starting position->" + staringPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, staringPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
    }

    private Optional<AutoModeBase> getAutoModeForParams(final DesiredMode mode, final StartingPosition position) {   
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case RENDEZVOUS:
                switch (position){
                    case MID:
                        return Optional.of(new MidToRendAndShootMode());
                    case RIGHT:
                        return Optional.of(new RightToRendAndShootMode());
                    case LEFT:
                        return Optional.of(new LeftToRendAndShootMode());
                    default:
                        break;
                }
                break;
            case TRENCH:
                switch(position){
                    case MID:
                        return Optional.of(new MidToTrenchAndShootMode());
                    case RIGHT:
                        return Optional.of(new RightToTrenchAndShootMode());
                    case LEFT:
                        return Optional.of(new LeftToTrenchAndShootMode());
                }
            case SHOOT:
                return Optional.of(new SteerAndShootAction(position));
            case TEST:
                return Optional.of(new TestMode());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

}
