package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="pedroTest2")
public class PedroPathShell extends OpMode {
// declare follower & path timer
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public HardwareManager hwMgr = new HardwareManager(hardwareMap);

// create states by paths & actions
    public enum PathState {
         DRIVE_START_TO_FIRST_POSITION,
        DRIVE_TO_SECOND_POSITION,
        ACTION_FIRST_POSITION,
        DRIVE_TO_END_POSITION,
            PARK
    }

    private PathState pathState;

    // declare poses for each path position
    private final Pose startPosition = new Pose(56,8,Math.toRadians(90));
    private final Pose firstPosition = new Pose(56,36,Math.toRadians(90));
    private final Pose secondPosition = new Pose(23.51864030784161,47.06881842120114);

    private final Pose endPosition = new Pose (54.61318226639513, 100.49472548616492);

    // declare pathchains for each path
    private PathChain driveStartToFirst, driveFirstToSecond,driveSecondToEnd;

    // build paths for each path chain
    public void buildPaths(){
        driveStartToFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPosition,firstPosition))
                .setLinearHeadingInterpolation(startPosition.getHeading(),firstPosition.getHeading())
                .build();

        driveFirstToSecond = follower.pathBuilder()
                .addPath(new BezierLine(firstPosition,secondPosition))
                .setTangentHeadingInterpolation()
                .build();
        driveSecondToEnd = follower.pathBuilder()
                .addPath(new BezierLine(secondPosition,endPosition))
                .setTangentHeadingInterpolation()
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_START_TO_FIRST_POSITION:
                follower.followPath(driveStartToFirst,true);
                setPathState(PathState.ACTION_FIRST_POSITION);

               break;
            case ACTION_FIRST_POSITION:
                if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 5){
                    telemetry.addLine("first action");
                    setPathState(PathState.DRIVE_TO_SECOND_POSITION);
                                    }
                break;
            case DRIVE_TO_SECOND_POSITION:
                if (!follower.isBusy()){
                    follower.followPath(driveFirstToSecond,true);
                    setPathState(PathState.DRIVE_TO_END_POSITION);
                }
                break;

                
            case DRIVE_TO_END_POSITION:
                if (!follower.isBusy()){
                    follower.followPath(driveSecondToEnd);
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
