package com.team4.robot.paths.trench;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class MiddleTrenchPath2 implements PathContainer {
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(-275, -145, 0, 0));
        sWaypoints.add(new Waypoint(-230, -145, 15, 80));
        sWaypoints.add(new Waypoint(-160, -120, 5, 80));
        sWaypoints.add(new Waypoint(-120, -80, 5, 100));
        sWaypoints.add(new Waypoint(-100, -80, 0, 100));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-275, -145, Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}