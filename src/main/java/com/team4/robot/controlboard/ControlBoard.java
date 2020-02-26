package com.team4.robot.controlboard;

public class ControlBoard implements IControlBoard{
    private static ControlBoard instance = null;

    public static ControlBoard getInstance(){
        if(instance == null){
            instance = new ControlBoard();
        }
        return instance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IOperatorControlBoard mOperatorControlBoard;

    private ControlBoard(){
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        mOperatorControlBoard = GamepadOperatorControlBoard.getInstance();
    }

    @Override
    public boolean getIntake() {
        return mOperatorControlBoard.getIntake();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getShoot() {
        return mOperatorControlBoard.getShoot();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getVisionEnable() {
        return mDriveControlBoard.getVisionEnable();
    }

    @Override
    public boolean getReadyToManipulateControlPanel() {
        return mOperatorControlBoard.getReadyToManipulateControlPanel();
    }

    @Override
    public boolean getUpdateControlPanelMode() {
        return mOperatorControlBoard.getUpdateControlPanelMode();
    }

    @Override
    public boolean getKillCommand() {
        return mOperatorControlBoard.getKillCommand();
    }

    @Override
    public boolean getPauseSong() {
        return mOperatorControlBoard.getPauseSong();
    }

    @Override
    public boolean getStopSong() {
        return mOperatorControlBoard.getStopSong();
    }

    @Override
    public boolean getPrevSong() {
        return mOperatorControlBoard.getPrevSong();
    }

    @Override
    public boolean getNextSong() {
        return mOperatorControlBoard.getNextSong();
    }

    public boolean getCompressor(){
        return mOperatorControlBoard.getCompressor();
    }

    @Override
    public boolean getDropIntake() {
        return mDriveControlBoard.getDropIntake();
    }

    @Override
    public boolean getUpIntake() {
        return mDriveControlBoard.getUpIntake();
    }
}