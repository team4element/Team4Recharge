package com.team4.robot.auto.modes;

import java.util.Arrays;

import com.team4.lib.actionbase.ParallelAction;
import com.team4.lib.actionbase.SeriesAction;
import com.team4.lib.actionbase.WaitAction;
import com.team4.lib.autobase.AutoModeBase;
import com.team4.lib.autobase.AutoModeEndedException;
import com.team4.lib.path.PathContainer;
import com.team4.robot.actions.DrivePathAction;
import com.team4.robot.actions.DropIntakeAction;
import com.team4.robot.actions.IntakeThroughPathAction;
import com.team4.robot.actions.ResetPoseFromPathAction;
import com.team4.robot.actions.ShootAction;
import com.team4.robot.actions.WaitForPathMarkerAction;
import com.team4.robot.paths.rendezvous.MiddleRendPath1;
import com.team4.robot.paths.rendezvous.MiddleRendPath2;
import com.team4.robot.subsystems.Superstructure;
import com.team4.robot.subsystems.states.superstructure.SuperstructureState;

import edu.wpi.first.wpilibj.Timer;

public class MidToRendAndShootMode extends AutoModeBase{
    
    PathContainer path1;
    PathContainer path2;

    public MidToRendAndShootMode(){
        path1 = new MiddleRendPath1();
        path2 = new MiddleRendPath2();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        
        
        final double startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseFromPathAction(path1));
        // runAction(new ParallelAction(Arrays.asList(new DropIntakeAction(), new ShootAction(4))));
        runAction(new DrivePathAction(path1));
        // runAction(new ParallelAction(Arrays.asList(new DrivePathAction(path1), 
        // new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("Start Intake"),
        // new IntakeThroughPathAction(false))))));

        runAction(new WaitAction(.25));
        Superstructure.getInstance().setControlState(SuperstructureState.IDLE);
        runAction(new DrivePathAction(path2));
        runAction(new WaitAction(.25));
        runAction(new ShootAction(3));

        System.out.println(Timer.getFPGATimestamp() - startTime);

        /**
         * TODO: test if necessary or if causes locking on teleop
         * 
         * To prevent any unknown movement robot waits for 15 seconds after auto mode is complete during the auto period
         * and will not carry over to tele-op {@see Robot}
         */
        runAction(new WaitAction(15));
    }
}