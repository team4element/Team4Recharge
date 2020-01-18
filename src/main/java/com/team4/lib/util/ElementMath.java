package com.team4.lib.util;

/**
 * General math equations we use
 */

public class ElementMath {

	static double prevArrVal;

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

	
	public static double inchesToRotations(double inches, double circumference, double gearRatio) {
		return inches / ((circumference * gearRatio));

	}
	public static double rotationsToInches(double rotations, double circumference, double gearRatio) {
		return rotations * ((circumference * gearRatio));

	}

	public static double radiansPerSecondToTicksPer100ms(double rad_s, double encoderPPR) {
        return rad_s / (Math.PI * 2.0) * encoderPPR / 10.0;
    }

	public static double rotationsToTicks(double rotations, double ppr){
		return rotations * ppr;
	}

	public static double ticksToRotations(double ticks, double ppr){
		return ticks / ppr;
	}

	public static double tickPer100msToRPM(double ticks, double ppr){
		return ticksToRotations(ticks, ppr) * 600;
	}

	public static double tickPer100msToScaledRPM(double ticks, double ppr, double gearRatio){
		return scaleRPM(ticksToRotations(ticks * 600, ppr), gearRatio);
	}

	public static double rpmToTicksPer100ms(double rpm, double ppr){
		return rotationsToTicks(rpm / 600, ppr);
	}

	public static double scaleRPM(double initRPM, double gearRatio){
		return initRPM / gearRatio;
	}

	public static double unscaleRPM(double finRPM, double gearRatio){
		return finRPM * gearRatio;
	}

	/**
	public static double[] addElementToArray(double[] arr, double a){
		
		arr = Arrays.copyOf(arr, arr.length + 1);

		arr[arr.length-1] = a;

		return arr;
	}

	public static double[][] addArrayToMultArray(double[][] arr, double[] a){
		
		arr = Arrays.copyOf(arr, arr.length + 1);

		arr[arr.length-1] = a;
		

		return arr;
	}*/

}
