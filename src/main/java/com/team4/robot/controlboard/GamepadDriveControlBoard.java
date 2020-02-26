package com.team4.robot.controlboard;

import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.XboxController.Button;

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
    public boolean getQuickTurn() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }

    @Override
    public boolean getDropIntake() {
        return mController.getButton(Button.A);
    }

    @Override
    public boolean getUpIntake() {
        return mController.getButton(Button.B);
    }
}