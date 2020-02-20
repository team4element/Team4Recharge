package com.team4.robot.paths.rendezvous;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class RightRendPath1 implements PathContainer{
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(113, 145, 0, 0));
        sWaypoints.add(new Waypoint(160, 145, 15, 50, "Start Intake"));
        sWaypoints.add(new Waypoint(255, 50, 5, 70));
        sWaypoints.add(new Waypoint(225, 20, 5, 70));
        sWaypoints.add(new Waypoint(250, -25, 0, 70));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(113, 145, Rotation2d.fromDegrees(180));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}