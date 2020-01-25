package com.team4.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team4.robot.paths.waypoints.LeftStartToRendPoints;
import com.team4.robot.paths.waypoints.LeftStartToTrenchPoints;
import com.team4.robot.paths.waypoints.MidStartToRendPoints;
import com.team4.robot.paths.waypoints.MidStartToTrenchPoints;
import com.team4.robot.paths.waypoints.RightStartToRendPoints;
import com.team4.robot.paths.waypoints.RightStartToTrenchPoints;
import com.team4.robot.planners.DriveMotionPlanner;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 130.0; //was 130 ips, but lowered after testing
    private static final double kMaxAccel = 130.0; // same as Velocity
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 60.0;
    private static final double kFirstPathMaxVel = 60.0;


    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    public static final Pose2d kLevelOneStartPose = new Pose2d(new Translation2d(40.0, -50.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kOffHabPlatform = new Pose2d(new Translation2d(80.0, -50.0), Rotation2d.fromDegrees(180.0));

    public static final Pose2d kLeavepoint1 = new Pose2d(new Translation2d(205.0, -10.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLeavePoint2 = new Pose2d(new Translation2d(165.0, -10.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLeavePoint3 = new Pose2d(new Translation2d(100.0, -53.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kStartPose = new Pose2d(new Translation2d(65.0, -53.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kStartPoseComp = new Pose2d(new Translation2d(20, -50.0), Rotation2d.fromDegrees(180));
    public static final Pose2d kSecondPoseComp = new Pose2d(new Translation2d(70, -50.0), Rotation2d.fromDegrees(180));
    public static final Pose2d kThirdPoseComp = new Pose2d(new Translation2d(100, -10), Rotation2d.fromDegrees(180.0 + 90.0));
    public static final Pose2d kFinalPoseComp = new Pose2d(new Translation2d(100, 50), Rotation2d.fromDegrees(180 + 90));

    public static final Pose2d kStartPoseRet = new Pose2d(new Translation2d(140, 50), Rotation2d.fromDegrees(180 - 90));
    public static final Pose2d kSecondPoseRet = new Pose2d(new Translation2d(140, 0), Rotation2d.fromDegrees(180.0 - 90.0));
    public static final Pose2d kThirdPoseRet = new Pose2d(new Translation2d(110, -15.0), Rotation2d.fromDegrees(0));
    public static final Pose2d kFinalPoseRet = new Pose2d(new Translation2d(5, -15.0), Rotation2d.fromDegrees(0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        //All trajectories used when testing the bot
        public final MirroredTrajectory sideStartToNearScale;
        public final MirroredTrajectory returnToStart;
        public final MirroredTrajectory compPath;
        public final MirroredTrajectory retPath;

        //All comp trajectories
        public final MirroredTrajectory midStartToRendAndShoot;
        public final MirroredTrajectory midStartToTrenchAndShoot;
        public final MirroredTrajectory rightStartToRendAndShoot;
        public final MirroredTrajectory rightStartToTrenchAndIntake;
        public final MirroredTrajectory rightTrenchToTrenchAndShoot;
        public final MirroredTrajectory leftStartToRendAndShoot;
        public final MirroredTrajectory leftStartToTrenchAndIntake;
        public final MirroredTrajectory leftTrenchToTrenchAndShoot;

        //TODO: Rend paths need repair for comp bot, could mean having to make more paths
        private TrajectorySet() {
            //creates all test trajectories
            sideStartToNearScale = new MirroredTrajectory(getSideStartToNearScale());
            returnToStart = new MirroredTrajectory(getReturnToStart());
            compPath = new MirroredTrajectory(getComplexTrajcetory());
            retPath = new MirroredTrajectory(getComplexReturnPath());

            //create all comp trajectories
            midStartToRendAndShoot = new MirroredTrajectory(getRendPath());
            midStartToTrenchAndShoot = new MirroredTrajectory(getMidTrenchPath());
            rightStartToRendAndShoot = new MirroredTrajectory(getRightRendPath());
            rightStartToTrenchAndIntake = new MirroredTrajectory(getRightTrenchPath1());
            rightTrenchToTrenchAndShoot = new MirroredTrajectory(getRightTrenchPath2());
            leftStartToRendAndShoot = new MirroredTrajectory(getLeftRendPath());
            leftStartToTrenchAndIntake = new MirroredTrajectory(getLeftTrenchPath1());
            leftTrenchToTrenchAndShoot = new MirroredTrajectory(getLeftTrenchPath2());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearScale() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(105.0, -10.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(165.0, -10.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 0, 20.0,
            kMaxVelocity, kMaxAccel, kMaxVoltage);
            }

        private Trajectory<TimedState<Pose2dWithCurvature>> getReturnToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeavepoint1);
            waypoints.add(kLeavePoint2);
            waypoints.add(kLeavePoint3);
            waypoints.add(kStartPose);
        
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getComplexTrajcetory(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kStartPoseComp);
            waypoints.add(kSecondPoseComp);
            waypoints.add(kThirdPoseComp);
            waypoints.add(kFinalPoseComp);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getComplexReturnPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kStartPoseRet);
            waypoints.add(kSecondPoseRet);
            waypoints.add(kThirdPoseRet);
            waypoints.add(kFinalPoseRet);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRendPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(MidStartToRendPoints.startPose);
            waypoints.add(MidStartToRendPoints.allignIntakePose);
            waypoints.add(MidStartToRendPoints.beginIntakePose);
            waypoints.add(MidStartToRendPoints.finishIntakePose);
            // waypoints.add(MidStartToRendPoints.allignShootPose);
            waypoints.add(MidStartToRendPoints.finalPose);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getMidTrenchPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(MidStartToTrenchPoints.startPose);
            waypoints.add(MidStartToTrenchPoints.allignIntakePose);
            waypoints.add(MidStartToTrenchPoints.startIntakePose);
            
            //Theoretically would shoot from this
            waypoints.add(MidStartToTrenchPoints.finishIntakePose);
            
            // waypoints.add(MidStartToTrenchPoints.allignShootPose);
            // waypoints.add(MidStartToTrenchPoints.finalPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)), 
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightRendPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(RightStartToRendPoints.startPose);
            waypoints.add(RightStartToRendPoints.startIntake1stPose);
            waypoints.add(RightStartToRendPoints.startIntake2ndPose);
            waypoints.add(RightStartToRendPoints.finishIntakePose);
            waypoints.add(RightStartToRendPoints.finalPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
             kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightTrenchPath1(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(RightStartToTrenchPoints.startPose1);
            waypoints.add(RightStartToTrenchPoints.finishIntakePhase1);
            waypoints.add(RightStartToTrenchPoints.finishPose1);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightTrenchPath2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(RightStartToTrenchPoints.startPose2);
            waypoints.add(RightStartToTrenchPoints.allignShootPose);
            waypoints.add(RightStartToTrenchPoints.finishPose2);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftRendPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(LeftStartToRendPoints.startPose);
            waypoints.add(LeftStartToRendPoints.beginIntakePose);
            waypoints.add(LeftStartToRendPoints.finishIntakePose);
            waypoints.add(LeftStartToRendPoints.finalPose);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftTrenchPath1(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(LeftStartToTrenchPoints.startPose1);
            waypoints.add(LeftStartToTrenchPoints.beginIntakePose);
            waypoints.add(LeftStartToTrenchPoints.finishIntakePose);
            waypoints.add(LeftStartToTrenchPoints.finalPose1);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity,kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftTrenchPath2(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(LeftStartToTrenchPoints.startPose2);
            waypoints.add(LeftStartToTrenchPoints.allignPose);
            waypoints.add(LeftStartToTrenchPoints.finalPose2);
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
