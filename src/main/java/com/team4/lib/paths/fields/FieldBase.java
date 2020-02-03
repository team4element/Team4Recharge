package com.team4.lib.paths.fields;

import com.team254.lib.geometry.Translation2d;

/**
 * Look at Reference Image to figure where everything is
 */

public abstract class FieldBase{
    double mBlueBackWallToAutoLine;
    double mBlueSideWallToTrench;
    double mBlueBackWallToTrench;
    double mBlueBackWallToRendTop;
    double mBlueBackWallToRendBottom;
    double mBlueSideWallToRend;
    double mBlueSideToTriangle;
    double mBlueRendEdgeToEdge;
    double mBlueTrenchWidth;
    double mBlueTrenchBackBallTop;
    double mBlueTrenchBackBallBottom;

    double mRedBackWallToAutoLine;
    double mRedSideWallToTrench;
    double mRedBackWallToTrench;
    double mRedFrontWallToRend;
    double mRedBackWallToRend;
    double mRedSideWallToRend;
    double mRedSideWallToTriangle;
    double mRedRendEdgeToEdge;

    double mRendDiagonal;
    double mSideToSideWall; // Divide this by 2, will give center of field (waypoints based off center of field)

    public Translation2d getBlueTrenchFrontEdge(){
        return new Translation2d(mBlueBackWallToTrench, mSideToSideWall - mBlueSideWallToTrench);
    }

    public Translation2d getBlueTrenchFrontCenter(){
        return new Translation2d(mBlueBackWallToTrench, mSideToSideWall - (mBlueSideWallToTrench/2));
    }

    public Translation2d getBlueAutoLineMiddle(){
        return new Translation2d(mBlueBackWallToAutoLine, mSideToSideWall - mBlueSideToTriangle);
    }
    public Translation2d getBlueAutoLineRight(){
        return new Translation2d(mBlueBackWallToAutoLine, mBlueSideWallToTrench/2);
    }

    public Translation2d getBlueAutoLineLeft(){
        return new Translation2d(mBlueBackWallToAutoLine, mSideToSideWall/2);
    }

    public Translation2d getBlueRendEdge(){
        return new Translation2d(mBlueBackWallToRendTop + (mRendDiagonal/2), mSideToSideWall - (mBlueSideWallToRend + (mRendDiagonal/2)));
    }

    public Translation2d getBlueRendCenter(){
        return new Translation2d((mBlueBackWallToRendTop + mBlueBackWallToRendBottom)/2, (mRendDiagonal/2));
    }

    public Translation2d getBlueTrenchBackBallBottom(){
        return new Translation2d(mBlueBackWallToTrench + mBlueTrenchWidth, mSideToSideWall - (mBlueSideWallToTrench - mBlueTrenchBackBallBottom));
    }

    public Translation2d getBlueTrenchBackBallTop(){
        return new Translation2d(mBlueBackWallToTrench + mBlueTrenchWidth, mSideToSideWall - (mBlueSideWallToTrench - mBlueTrenchBackBallTop));
    }

    public Translation2d getRedTrenchFrontEdge(){
        return new Translation2d(mBlueBackWallToTrench, mSideToSideWall - mBlueSideWallToTrench);
    }

    public Translation2d getRedTrenchFrontCenter(){
        return new Translation2d(mBlueBackWallToTrench, mSideToSideWall - (mBlueSideWallToTrench/2));
    }

    public Translation2d getRedAutoLineMiddle(){
        return new Translation2d(mBlueBackWallToAutoLine, mSideToSideWall - mBlueSideToTriangle);
    }
    public Translation2d getRedAutoLineRight(){
        return new Translation2d(mBlueBackWallToAutoLine, mBlueSideWallToTrench/2);
    }

    public Translation2d getRedAutoLineLeft(){
        return new Translation2d(mBlueBackWallToAutoLine, mSideToSideWall/2);
    }

    public Translation2d getRedRendEdge(){
        return new Translation2d(mBlueBackWallToRendTop + (mRendDiagonal/2), mSideToSideWall - (mBlueSideWallToRend + (mRendDiagonal/2)));
    }

    public Translation2d getRedRendCenter(){
        return new Translation2d((mBlueBackWallToRendTop + mBlueBackWallToRendBottom)/2, (mRendDiagonal/2));
    }

    public Translation2d getRedTrenchBackBallBottom(){
        return new Translation2d(mBlueBackWallToTrench + mBlueTrenchWidth, mSideToSideWall - (mBlueSideWallToTrench - mBlueTrenchBackBallBottom));
    }

    public Translation2d getRedTrenchBackBallTop(){
        return new Translation2d(mBlueBackWallToTrench + mBlueTrenchWidth, mSideToSideWall - (mBlueSideWallToTrench - mBlueTrenchBackBallTop));
    }
}