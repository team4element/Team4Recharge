package com.team4.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team4.robot.constants.AutoConstants;
import com.team4.robot.constants.Constants;

import edu.wpi.first.wpilibj.DriverStation;

public class TalonUtil {
    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }

    /**
     * Configure talon fx, used for subsystems that use talon FX
     * 
     * @param talon Talon FX to be configured
     * @param master is the master talon
     */

    public static void configureTalonFX(TalonFX talon, boolean master) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.
                IntegratedSensor, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + ("Motor") + " encoder: " + sensorPresent, false);
        }
        talon.setSensorPhase(true);
        
        // if(master){
            // talon.enableVoltageCompensation(true);
            // talon.configVoltageCompSaturation(12.0, IOConstants.kLongCANTimeoutMs);
            talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
            talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
            //Most likely same for all subsystems, so sticking to drive temporarily
            talon.configClosedloopRamp(AutoConstants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        // }
        
        talon.configNeutralDeadband(0.04, 0);
            
    }

        /**
     * Configure talon fx, used for subsystems that use talon FX
     * 
     * @param talon Talon FX to be configured
     * @param master is the master talon
     * @param left is the left side or not
     */

    public static void configureTalonFX(TalonFX talon, boolean left, boolean master) {
        // talon.setInverted(!left);
        talon.setInverted(left ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
        configureTalonFX(talon, master);
    }

    public static void configureTalonSRX(TalonSRX talon) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.
                CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + " encoder: " + sensorPresent, false);
        }
        talon.setSensorPhase(true);
        // talon.enableVoltageCompensation(true);
        // talon.configVoltageCompSaturation(12.0, IOConstants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(AutoConstants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    public static void configureTalonSRX(TalonSRX talon, boolean left) {
        talon.setInverted(!left);
        configureTalonSRX(talon);
    }

    private static boolean mIsBrakeMode;

    public static void setBrakeMode(BaseTalon talon, boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            talon.setNeutralMode(mode);
            talon.setNeutralMode(mode);
        }
    }
}