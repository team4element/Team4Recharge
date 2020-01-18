package com.team4.robot.subsystems.states;

// The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID 
        TURN_TO_HEADING,
        DRIVE_VELOCITY
    }