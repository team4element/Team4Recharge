package com.team4.robot.paths.trench;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class MiddleTrenchPath1 implements PathContainer{
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(-105, -75, 0, 0));
        sWaypoints.add(new Waypoint(-130, -75, 5, 70, "Start Intake"));
        sWaypoints.add(new Waypoint(-200, -115, 5, 70));
        sWaypoints.add(new Waypoint(-260, -145, 0, 80));
        sWaypoints.add(new Waypoint(-305, -145, 0, 80));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-105, -75, Rotation2d.fromDegrees(180));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}