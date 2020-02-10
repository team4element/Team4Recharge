package com.team4.robot.paths.rendezvous;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathContainer;

public class LeftRendPath2 implements PathContainer{

    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(240 , -15, 0, 0));
        sWaypoints.add(new Waypoint(190, -15, 5, 80));
        sWaypoints.add(new Waypoint(190, 25, 15, 80));
        sWaypoints.add(new Waypoint(175, 65, 5, 100));
        sWaypoints.add(new Waypoint(130, 65, 0, 100));        
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(240, -15, Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }

}