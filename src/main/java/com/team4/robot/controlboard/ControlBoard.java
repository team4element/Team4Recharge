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
    public boolean getMoveConveyor() {
        return mOperatorControlBoard.getMoveConveyor();
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
}