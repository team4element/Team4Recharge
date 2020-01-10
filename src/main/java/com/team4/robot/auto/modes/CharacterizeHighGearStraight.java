package com.team4.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.physics.DriveCharacterization;
import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.CollectAccelerationData;
import com.team4.robot.actions.CollectVelocityData;

public class CharacterizeHighGearStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        // runAction(new ShiftHighGearAction(false));
        // runAction(new WaitAction(10));

        runAction(new CollectVelocityData(velocityData, false, false));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, true, false));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}