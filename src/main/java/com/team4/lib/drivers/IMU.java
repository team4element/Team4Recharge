package com.team4.lib.drivers;

public interface IMU {

	double getFusedHeading();

	double getRawYawDegrees();

	double getPitch();

	double getRoll();

	boolean isPresent();

	boolean reset();
}