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

@Autonomous
public class PedroTestShell extends OpMode {
// declare follower & path timer
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    public HardwareManager hwMgr = new HardwareManager(hardwareMap);

// create states by paths & actions
    public enum PathState {
         DRIVE_START_TO_FIRSTPOSITION,
        DRIVE_TO_SECONDPOSITION,
        ACTION_FIRST_POSITION,
        DRIVE_TO_ENDPOSITION,
        PARK
    }

    private PathState pathState;

    // declare poses for each path position
    private final Pose startPosition = new Pose(94.50173310225304,3.773830155979213,Math.toRadians(90));
    private final Pose firstPosition = new Pose(94.50173310225304,46.83968804159445,Math.toRadians(90));
    private final Pose secondPosition = new Pose(70.51707362133854,70.40775472944924);
    private final Pose endPosition = new Pose(94.75800391781453,93.79117400061122);

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
            case DRIVE_START_TO_FIRSTPOSITION:
                follower.followPath(driveStartToFirst,true);
                setPathState(PathState.ACTION_FIRST_POSITION);

               break;
            case ACTION_FIRST_POSITION:
                if (!follower.isBusy()  && pathTimer.getElapsedTimeSeconds() > 5){
                    telemetry.addLine("first action");
                    setPathState(PathState.DRIVE_TO_SECONDPOSITION);
                                    }
                break;
            case DRIVE_TO_SECONDPOSITION:
                if (!follower.isBusy()){
                    follower.followPath(driveFirstToSecond,true);
                    setPathState(PathState.DRIVE_TO_ENDPOSITION);
                }
                break;
            case DRIVE_TO_ENDPOSITION:
                if (!follower.isBusy()){
                    follower.followPath(driveSecondToEnd);
                    setPathState(PathState.PARK);
                }
                break;
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
        pathState = PathState.DRIVE_START_TO_FIRSTPOSITION;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        hwMgr.init(hardwareMap);
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
