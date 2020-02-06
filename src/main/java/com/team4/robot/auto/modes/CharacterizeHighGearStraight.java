package com.team4.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.physics.DriveCharacterization;
import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.CollectAccelerationDataAction;
import com.team4.robot.actions.CollectVelocityDataAction;

public class CharacterizeHighGearStraight extends AutoModeBase {
    private final boolean reverse;
    private final boolean turn;

    public CharacterizeHighGearStraight() {
        reverse = false;
        turn = false;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.DataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.DataPoint> accelerationData = new ArrayList<>();

        runAction(new CollectVelocityDataAction(velocityData, reverse, turn));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationDataAction(accelerationData, reverse, turn));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}