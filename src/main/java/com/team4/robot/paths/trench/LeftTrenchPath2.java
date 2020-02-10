package com.team4.robot.paths.trench;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class LeftTrenchPath2 implements PathContainer{
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(395, 135, 0, 0)); //Change X to 395
        sWaypoints.add(new Waypoint(370, 140, 5, 50)); // Change X to 370
        sWaypoints.add(new Waypoint(345, 145, 5, 70)); // Change X to 345
        sWaypoints.add(new Waypoint(220, 145, 0, 70)); //Change X to 220
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(395, 135, Rotation2d.fromDegrees(0d));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}