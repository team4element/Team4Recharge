package com.team4.robot.paths.trench;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class LeftTrenchPath1 implements PathContainer{
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(-105, 0, 0, 0)); //
        sWaypoints.add(new Waypoint(-180, -145, 15, 50)); 
        sWaypoints.add(new Waypoint(-225, -145, 0, 70));
        sWaypoints.add(new Waypoint(-345, -145, 5, 70));
        sWaypoints.add(new Waypoint(-370, -140, 5, 70));
        sWaypoints.add(new Waypoint(-395, -135, 0, 70));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(-105, 0, Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}