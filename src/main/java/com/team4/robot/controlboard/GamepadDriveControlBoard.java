package com.team4.robot.controlboard;

import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.XboxController.Button;
import com.team4.robot.controlboard.XboxController.Side;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveController);
    }

    @Override
    public double getThrottle() {
        return  -mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getTurn() {
        return mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean getVisionEnable() {
        return mController.getButton(Button.LB);
    }

    @Override
    public boolean getDropIntake() {
        return mController.getButton(Button.RB);
    }

    @Override
    public boolean getIntake() {
        return mController.getTrigger(Side.RIGHT);
    }

    @Override
    public boolean getRotationControl() {
        return mController.getButton(Button.A);
    }

    @Override
    public boolean getPositionControl() {
        return mController.getButton(Button.B);
    }

    @Override
    public boolean getWheelUp() {
        return mController.getDPad() == 0;
    }

    @Override
    public boolean getWheelDown() {
        return mController.getDPad() == 180;
    }

    @Override
    public boolean getClimbUp() {
        return mController.getButton(Button.X);
    }

    @Override
    public boolean getClimbDown() {
        return mController.getButton(Button.Y);
    }

    @Override
    public boolean getWinch() {
        return mController.getButton(Button.START);
    }

}