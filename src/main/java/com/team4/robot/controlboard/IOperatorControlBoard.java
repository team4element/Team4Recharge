package com.team4.robot.controlboard;

public interface IOperatorControlBoard{
    boolean getShoot();

    boolean getBackConvey();

    boolean getVisionOverride();

    boolean getReverseIntake();

    boolean getReverseConveyor();

    boolean getReverseSuperstructure();

    boolean getClimbUp();

    boolean getClimbDown();

    boolean getWinch();

    boolean getCompress();
}