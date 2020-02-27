package com.team4.robot.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getVisionEnable();

    boolean getDropIntake();

    boolean getIntake();

    boolean getRotationControl();
    
    boolean getPositionControl();

    boolean getWheelUp();

    boolean getWheelDown();

    boolean getClimbUp();

    boolean getClimbDown();

    boolean getWinch();
}