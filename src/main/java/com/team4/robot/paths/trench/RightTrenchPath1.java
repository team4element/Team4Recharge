package com.team4.robot.paths.trench;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class RightTrenchPath1 implements PathContainer{
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(105, 145, 0, 0));
        sWaypoints.add(new Waypoint(160, 145, 0, 50, "Start Intake")); //change X to 205
        sWaypoints.add(new Waypoint(220, 145, 5, 70)); // change X to 345
        sWaypoints.add(new Waypoint(245, 140, 5, 70)); // change X to 370
        sWaypoints.add(new Waypoint(270, 135, 0, 70));// change X to 395
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return null;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}