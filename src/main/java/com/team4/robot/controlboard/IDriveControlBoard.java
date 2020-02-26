package com.team4.robot.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getVisionEnable();

    boolean getDropIntake();
    
    boolean getUpIntake();
    
}