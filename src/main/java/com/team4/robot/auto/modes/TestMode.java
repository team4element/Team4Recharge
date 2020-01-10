package com.team4.robot.auto.modes;

import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.robot.actions.AutoSteerAndDistanceAction;
import com.team4.robot.actions.DriveTrajectory;
import com.team4.robot.actions.TurnToHeadingAction;
import com.team4.robot.paths.TrajectoryGenerator;
import com.team4.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class TestMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Test mode");
        Drive.getInstance().startLogging();
        double startTime =Timer.getFPGATimestamp();     
        // runAction(new DriveVelocity(2, 5));
        // runAction(new WaitAction(5));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().compPath.get(true), true));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(90)));
        runAction(new AutoSteerAndDistanceAction(15, 5));
        // runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180)));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(270.0)));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().retPath.get(true), true));
        runAction(new TurnToHeadingAction(Rotation2d.fromDegrees(180.0)));
        System.out.println(Timer.getFPGATimestamp()- startTime);
        Drive.getInstance().stopLogging();
    }
}
