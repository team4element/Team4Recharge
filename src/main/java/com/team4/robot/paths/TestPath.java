package com.team4.robot.paths;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team4.lib.path.PathBuilder;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.PathContainer;

public class TestPath implements PathContainer{
    
    @Override
    public Path buildPath() {
        List<Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(100, 0, 0, 0)); //Change X to 395
        sWaypoints.add(new Waypoint(180, 0, 0, 50)); // Change X to 370
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(100, 0, Rotation2d.fromDegrees(180.0));
    }
 
@Override
public boolean isReversed() {
    return false;
}

}