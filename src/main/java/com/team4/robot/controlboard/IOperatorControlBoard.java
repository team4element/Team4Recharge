package com.team4.robot.controlboard;

public interface IOperatorControlBoard{
    boolean getShoot();

    boolean getMoveConveyor();

    boolean getUpdateControlPanelMode();

    boolean getReadyToManipulateControlPanel();

    boolean getKillCommand();
}