package com.team4.lib.util;

/**
 * General math equations we use 
 */

public class ElementMath {

	// Higher sensitivity on joystick
	public static double squareInput(double input) {
		return Math.pow(input, 2) * input/Math.abs(input);
	}

	// Higher sensitivity on joystick
	public static double cubeInput(double input) {
		return Math.pow(input, 3);
	}

	// Deadband for TeleOp Drive
	public static double handleDeadband(double val, double deadband){
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	public static boolean isWithinDeadband(double val, double deadband){
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
		boolean isTrue = false;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		if(Math.abs(val) > Math.abs(deadband)){
			isTrue = true;
		}else{
			isTrue = false;
		}
		return isTrue;
	}

	public static double ticksToInches(double ticks, double circumference, double gearRatio,
			double TicksPerRevolution) {
		return ticks * ((circumference * gearRatio) / TicksPerRevolution);

	}
	public static double ticksToInchesPerSec(double ticks, double circumference, double gearRatio,
	double TicksPerRevolution, double hunderedmsPerMinute) {
			return (ticks * ((circumference * gearRatio) / (TicksPerRevolution * hunderedmsPerMinute)))/60;

	}

	
	public static double inchesToRotations(double inches, double circumference, double gearRatio) {
		return inches / ((circumference * gearRatio));

	}
	public static double rotationsToInches(double rotations, double circumference, double gearRatio) {
		return rotations * ((circumference * gearRatio));

	}


}
