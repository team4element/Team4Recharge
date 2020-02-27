package com.team4.robot.controlboard;

import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.XboxController.Button;
import com.team4.robot.controlboard.XboxController.Side;

public class GamepadOperatorControlBoard implements IOperatorControlBoard{
    private static GamepadOperatorControlBoard instance = null;

    public static GamepadOperatorControlBoard getInstance(){
        if (instance == null){
            instance = new GamepadOperatorControlBoard();
        }
        return instance;
    }

    private final XboxController mController;

    private GamepadOperatorControlBoard(){
        mController = new XboxController(Constants.kOperatorController);
    }
    
    @Override
    public boolean getShoot() {
        return mController.getButton(Button.LB);
    }

    @Override
    public boolean getBackConvey() {
        return mController.getButton(Button.RB);
    }

    @Override
    public boolean getVisionOverride() {
        return mController.getTrigger(Side.LEFT);
    }

    @Override
    public boolean getReverseIntake() {
        return mController.getButton(Button.A);
    }

    @Override
    public boolean getReverseConveyor() {
        return mController.getButton(Button.B);
    }

    @Override
    public boolean getReverseSuperstructure() {
        return mController.getTrigger(Side.RIGHT);
    }

    @Override
    public boolean getClimbUp() {
        return mController.getButton(Button.X);
    }

    public boolean getClimbDown() {
        return mController.getButton(Button.Y);
    }

    @Override
    public boolean getWinch() {
        return mController.getButton(Button.START);
    }

    @Override
    public boolean getCompress() {
        return mController.getButton(Button.L_JOYSTICK);
    }
}