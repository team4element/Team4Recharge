package com.team4.lib.paths;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team4.lib.paths.fields.FieldBase;
import com.team4.lib.paths.fields.PracticeField;
import com.team4.lib.paths.fields.ReferenceField;

import edu.wpi.first.wpilibj.DriverStation;

public class PathAdapter{
    private static final DriverStation ds = DriverStation.getInstance();

    private static FieldBase kReferenceField = new ReferenceField();
    private static FieldBase kCurrentField = new PracticeField();

    private static Translation2d startPoseAdaptBlueMiddle = kCurrentField.getBlueAutoLineMiddle()
            .translateBy(kReferenceField.getBlueAutoLineMiddle().inverse());
    private static Translation2d startPoseAdaptRedMiddle = kCurrentField.getRedAutoLineMiddle()
            .translateBy(kReferenceField.getRedAutoLineMiddle().inverse());
    
    
    private static Translation2d startPoseAdaptBlueRight = kCurrentField.getBlueAutoLineRight()
            .translateBy(kReferenceField.getBlueAutoLineRight().inverse());
    private static Translation2d startPoseAdaptRedRight = kCurrentField.getRedAutoLineRight()
            .translateBy(kReferenceField.getRedAutoLineRight().inverse());

    private static Translation2d rendEdgeAdaptBlue = kCurrentField.getBlueRendEdge()
            .translateBy(kReferenceField.getBlueRendEdge().inverse());
    private static Translation2d rendEdgeAdaptRed = kCurrentField.getBlueRendEdge()
            .translateBy(kReferenceField.getBlueRendEdge().inverse());

    public static Pose2d adaptStartPointMiddle(final Pose2d point) {
        return new Pose2d(
                ds.getAlliance() == DriverStation.Alliance.Blue ? startPoseAdaptBlueMiddle : startPoseAdaptRedMiddle, 
                point.getRotation());
    }

    public static Pose2d adaptStartPointRight(final Pose2d point){
        return new Pose2d(
            ds.getAlliance() == DriverStation.Alliance.Blue ? startPoseAdaptBlueRight : startPoseAdaptRedRight, 
            point.getRotation());
    }

    public static Pose2d adaptRendFirstPoint(final Pose2d point){
        return new Pose2d(
            ds.getAlliance() == DriverStation.Alliance.Blue ? rendEdgeAdaptBlue : rendEdgeAdaptRed,
            point.getRotation());
    }

    public static Pose2d adaptTrenchFrontPoint(final Pose2d point){
        return Pose2d.identity();
    }

    public static Pose2d adaptRendFrontPoint(final Pose2d point){
        return Pose2d.identity();
    }

    public static Pose2d adaptTrenchBackPoint(final Pose2d point){
        return Pose2d.identity();
    }

    

}