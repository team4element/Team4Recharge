package com.team4.robot.paths.rendezvous;

import java.util.ArrayList;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathContainer;

public class MiddleRendPath1 implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        // sWaypoints.add(new Waypoint(0,0,0,0));
        // sWaypoints.add(new Waypoint(60,0,15,60));
        // sWaypoints.add(new Waypoint(80,30,15,60,"LEDOff"));
        // sWaypoints.add(new Waypoint(160,30,0,60));
        // sWaypoints.add(new Waypoint(0,0,0,0));
        // sWaypoints.add(new Waypoint(70,0,45,60));
        // sWaypoints.add(new Waypoint(100,60,15, 60));
        // sWaypoints.add(new Waypoint(100,80,0,60));
        sWaypoints.add(new Waypoint(113, 75, 0, 0));
        sWaypoints.add(new Waypoint(120, 75, 5, 60));
        sWaypoints.add(new Waypoint(135, 45, 15, 60, "Start Intake"));
        sWaypoints.add(new Waypoint(205, 20, 10, 70));
        sWaypoints.add(new Waypoint(240, -25, 0, 80, "Finish Intake"));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(new Translation2d(113, 75), Rotation2d.fromDegrees(180.0));
   
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}