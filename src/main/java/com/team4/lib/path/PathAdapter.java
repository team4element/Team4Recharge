package com.team4.lib.path;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team4.lib.path.PathBuilder.Waypoint;
import com.team4.lib.path.fields.FieldBase;
import com.team4.lib.path.fields.PracticeField;
import com.team4.lib.path.fields.ReferenceField;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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


    private static Translation2d rendCenterAdaptBlue = kCurrentField.getBlueRendCenter()
            .translateBy(kReferenceField.getBlueRendCenter().inverse());
    private static Translation2d rendCenterAdaptRed = kCurrentField.getRedRendCenter()
            .translateBy(kReferenceField.getRedRendCenter());

    private static Translation2d trenchEdgeAdaptBlue = kCurrentField.getBlueTrenchFrontEdge()
            .translateBy(kReferenceField.getBlueTrenchFrontEdge());
    private static Translation2d trenchEdgeAdaptRed = kCurrentField.getRedTrenchFrontEdge()
            .translateBy(kReferenceField.getRedTrenchFrontEdge());

    public static Waypoint adaptStartPointMiddle(Waypoint input) {
        if(ds.getAlliance() == Alliance.Blue){
                input.setPosition(input.getPosition().translateBy(startPoseAdaptBlueMiddle));
        }else{
                input.setPosition(input.getPosition().translateBy(startPoseAdaptRedMiddle));
        }
        return input;    
    }

    public static Waypoint adaptStartPointRight(Waypoint input){
        if(ds.getAlliance() == Alliance.Blue){
                input.setPosition(input.getPosition().translateBy(startPoseAdaptBlueRight));
        }else{
                input.setPosition(input.getPosition().translateBy(startPoseAdaptRedRight));
        }
        return input;    
    }

    public static Waypoint adaptRendFirstPoint(Waypoint input){
        if(ds.getAlliance() == Alliance.Blue){
                input.setPosition(input.getPosition().translateBy(rendEdgeAdaptBlue));
        }else{
                input.setPosition(input.getPosition().translateBy(rendEdgeAdaptRed));
        }
        return input;
    }



    public static Waypoint adaptTrenchFrontEdgePoint(Waypoint input){
        if(ds.getAlliance() == Alliance.Blue){
                input.setPosition(input.getPosition().translateBy(trenchEdgeAdaptBlue));
        }else{
                input.setPosition(input.getPosition().translateBy(trenchEdgeAdaptRed));
        }

        return input;
    }

    public static Waypoint adaptRendCenterPoint(Waypoint input){
        if(ds.getAlliance() == Alliance.Blue){
                input.setPosition(input.getPosition().translateBy(rendCenterAdaptBlue));
        }
                input.setPosition(input.getPosition().translateBy(rendCenterAdaptRed));
        
        return input;
    }

    public static Pose2d adaptTrenchBackPoint(final Pose2d point){
        return Pose2d.identity();
    }

    

}