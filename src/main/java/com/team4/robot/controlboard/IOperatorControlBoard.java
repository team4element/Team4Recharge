package com.team4.robot.controlboard;

public interface IOperatorControlBoard{
    boolean getShoot();

    boolean getIntake();

    boolean getUpdateControlPanelMode();

    boolean getReadyToManipulateControlPanel();

    boolean getKillCommand();

    boolean getPauseSong();

    boolean getStopSong();

    boolean getPrevSong();

    boolean getNextSong();
}