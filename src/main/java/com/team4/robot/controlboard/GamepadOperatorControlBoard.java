package com.team4.robot.controlboard;

import com.team4.robot.constants.Constants;
import com.team4.robot.controlboard.XboxController.Button;

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
    public boolean getMoveConveyor() {
        return mController.getButton(Button.RB);
    }

    @Override
    public boolean getUpdateControlPanelMode() {
        return mController.getButton(Button.Y);
    }

    @Override
    public boolean getReadyToManipulateControlPanel() {
        return mController.getButton(Button.X);
    }

    @Override
    public boolean getKillCommand() {
        return mController.getButton(Button.A);
    }
    
}