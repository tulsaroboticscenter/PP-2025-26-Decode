package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="pedroTest")
public class PedroPathTest1 extends OpMode {
// declare follower & path timer
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public HardwareManager hwMgr = new HardwareManager(hardwareMap);

// create states by paths & actions
    public enum PathState {
         DRIVE_START_TO_FIRST_POSITION,
        DRIVE_TO_SECOND_POSITION,
        REAR_SHOOT_ACTION,
        DRIVE_TO_END_POSITION,
    DRIVE_TO_SHOOT_POSITION_1,
    DRIVE_TO_SHOOT_POSITION_2,
    DRIVE_TO_INTAKE2,
        PARK
    }

    private PathState pathState;

    // declare poses for each path position
    private final Pose startPosition = new Pose(56,8,Math.toRadians(90));
    private final Pose firstPosition = new Pose(47.47394136807817,35.259771986970684,Math.toRadians(180));
    private final Pose secondPosition = new Pose(14.564609069440053,35.45605250635125);
    private final Pose shootPosition = new Pose(69.59771986970684,23.967426710097715);
    private final Pose intake2Position = new Pose (47.115782785471026, 58.891609084666584);
    private final Pose endPosition = new Pose (105.65161340436354, 30.315061853396223);

    // declare pathchains for each path
    private PathChain driveStartToFirst, driveFirstToSecond,driveSecondToShoot, driveShootToIntake2, driveIntake2ToShoot, driveShootToEnd;

    // build paths for each path chain
    public void buildPaths(){
        driveStartToFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPosition,firstPosition))
                .setLinearHeadingInterpolation(startPosition.getHeading(),firstPosition.getHeading())
                .build();

        driveFirstToSecond = follower.pathBuilder()
                .addPath(new BezierLine(firstPosition,secondPosition))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .build();
        driveSecondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPosition,shootPosition))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(90))
                .build();
        driveShootToIntake2 = follower.pathBuilder()
                .addPath((new BezierLine(shootPosition,intake2Position)))
                .setTangentHeadingInterpolation()
                .build();
        driveIntake2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(intake2Position, shootPosition))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(90))
                .build();
        driveShootToEnd = follower.pathBuilder()
                .addPath((new BezierLine(shootPosition, endPosition)))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_START_TO_FIRST_POSITION:
                follower.followPath(driveStartToFirst,true);
                setPathState(PathState.DRIVE_TO_SECOND_POSITION);

               break;
            case REAR_SHOOT_ACTION:
                if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 5){
                    telemetry.addLine("first action");
                    setPathState(PathState.DRIVE_TO_INTAKE2);
                                    }
                break;
            case DRIVE_TO_INTAKE2:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(driveShootToIntake2,true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POSITION_2);
                }
                break;
            case DRIVE_TO_SECOND_POSITION:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(driveFirstToSecond,true);
                    setPathState(PathState.DRIVE_TO_SHOOT_POSITION_1);
                }
                break;
            case DRIVE_TO_SHOOT_POSITION_1:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(driveSecondToShoot);
                    setPathState(PathState.REAR_SHOOT_ACTION);
                }
                break;
            case DRIVE_TO_SHOOT_POSITION_2:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(driveIntake2ToShoot);
                    setPathState(PathState.DRIVE_TO_END_POSITION);
                }
                break;
                
            case DRIVE_TO_END_POSITION:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3){
                    follower.followPath(driveShootToEnd);
                    setPathState(PathState.PARK);
                }
            case PARK:
                if (!follower.isBusy()){
                    stop();
                }
                break;
            default:
                telemetry.addLine("No state defined");
        }
    }

    public void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_TO_FIRST_POSITION;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        hwMgr.init_auto(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPosition);
    }

    @Override
    public void start() {
        super.start();
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();

        statePathUpdate();

        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}
