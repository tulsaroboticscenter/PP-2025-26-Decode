package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Near Gate CTS", group = "Autonomous", preselectTeleOp = "TeleOp")
public class BlueNearGateCTS extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw;

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    // All x-coordinates mirrored: x_blue = 192 - x_red
    // All non-vertical headings mirrored: heading_blue = π - heading_red
    private final Pose startPose  = new Pose(10, 133.117, Math.toRadians(180 - (-53.322)));  // ~233.3°
    private final Pose scorePose  = new Pose(35,    80,      Math.toRadians(180));
    private final Pose intake1    = new Pose(-5,     81,      Math.toRadians(180));
    private final Pose clearGate  = new Pose(0,     73,      Math.toRadians(180));
    private final Pose clearGate2 = new Pose(0,     73,      Math.toRadians(180));
    private final Pose gateControlPoint = new Pose(10, 71);
    private final Pose prepIntake2 = new Pose(25,  62.5,   Math.toRadians(180));
    private final Pose intake2    = new Pose(-5,     60,      Math.toRadians(180));
    private final Pose prepIntake3 = new Pose(25,  37.5,   Math.toRadians(180));
    private final Pose intake3    = new Pose(-5,     34,      Math.toRadians(180));
    private final Pose park       = new Pose(15 , 90,      Math.toRadians(-90));

    private PathChain scorePreload, parkPath, intakeLine1, scoreLine1, lineupIntake2, intakeLine2,
            clearGatePath2, scoreLine2, lineupIntake3, intakeLine3, scoreLine3, clearGatePath;

    public void buildPaths() {
        switch (numOfSpikes)
        {
            case 3:
                lineupIntake3 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, prepIntake3))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                intakeLine3 = follower.pathBuilder()
                        .addPath(new BezierLine(prepIntake3, intake3))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                scoreLine3 = follower.pathBuilder()
                        .addPath(new BezierLine(intake3, scorePose))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

            case 2:
                lineupIntake2 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, prepIntake2))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                intakeLine2 = follower.pathBuilder()
                        .addPath(new BezierLine(prepIntake2, intake2))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                clearGatePath2 = follower.pathBuilder()
                        .addPath(new BezierCurve(intake2, gateControlPoint, clearGate2))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                scoreLine2 = follower.pathBuilder()
                        .addPath(new BezierLine(clearGate2, scorePose))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

            case 1:
                intakeLine1 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, intake1))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                clearGatePath = follower.pathBuilder()
                        .addPath(new BezierCurve(intake1, gateControlPoint, clearGate))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                scoreLine1 = follower.pathBuilder()
                        .addPath(new BezierLine(intake1, scorePose))
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

            case 0:
                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, scorePose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                        .build();

            default:
        }
    }

    public void Park()
    {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), park))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        follower.followPath(parkPath);
    }

    public void autonomousPathBlueUpdate() {
        switch (pathState) {
            case 0:
                hw.turret.spinUpFlywheel();
                hw.intake.partialIntake();
                telemetry.addLine("spinning up flywheel-completed");

                follower.followPath(scorePreload, true);
                shooterTimer.resetTimer();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy())
                {
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 10)
                    {
                        hw.intake.forceIntake();
                    }
                    if (shooterTimer.getElapsedTime() > 2000)
                    {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        if (numOfSpikes > 0)
                        {
                            follower.followPath(intakeLine1, true);
                            setPathState(2);
                        }
                        else
                        {
                            Park();
                            setPathState(11);
                        }
                    }
                }
                else
                {
                    shooterTimer.resetTimer();
                }
                break;

            case 2:
                if (!follower.isBusy() || follower.isRobotStuck()) {
                    follower.followPath(clearGatePath);
                    setPathState(3);
                }

            case 3:
                if (!follower.isBusy()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    follower.followPath(scoreLine1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 500)
                    {
                        hw.intake.forceIntake();
                    }
                    if (shooterTimer.getElapsedTime() > 2000)
                    {
                        hw.intake.closeGate();
                        hw.intake.stop();
                        if (numOfSpikes > 1)
                        {
                            follower.followPath(lineupIntake2, true);
                            setPathState(5);
                        }
                        else
                        {
                            Park();
                            setPathState(11);
                        }
                    }
                }
                else {
                    shooterTimer.resetTimer();
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() || follower.isRobotStuck()) {
                    follower.followPath(clearGatePath2);
                    setPathState(7);
                }

            case 7:
                if (!follower.isBusy()) {
                    hw.intake.stop();
                    hw.turret.spinUpFlywheel();
                    follower.followPath(scoreLine2, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();
                    if (shooterTimer.getElapsedTime() > 500)
                    {
                        hw.intake.forceIntake();
                    }
                    if (shooterTimer.getElapsedTime() > 2000)
                    {
                        hw.intake.closeGate();
                        hw.intake.stop();
                        if (numOfSpikes > 2)
                        {
                            follower.followPath(lineupIntake3, true);
                            setPathState(9);
                        }
                        else
                        {
                            Park();
                            setPathState(12);
                        }
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine3, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    hw.intake.stop();
                    hw.turret.spinUpFlywheel();
                    follower.followPath(scoreLine3, true);
                    shooterTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();
                    if (shooterTimer.getElapsedTime() > 500)
                    {
                        hw.intake.forceIntake();
                    }
                    if (shooterTimer.getElapsedTime() > 2000)
                    {
                        hw.intake.closeGate();
                        shooterTimer.resetTimer();
                        Park();
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy() && shooterTimer.getElapsedTimeSeconds() > 2)
                {
                    setPathState(-1);
                    stop();
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathBlueUpdate();
        telemetry.addLine("BLUE");

        hw.lights.update();
        hw.turret.update();
        hw.intake.update();

        hw.turret.setTarget(follower.getPose(), goalPosition);
        hw.turret.updateFlywheelAndHood(follower.getPose(), goalPosition);

        Field.lastKnownPosition = new Pose2D(DistanceUnit.INCH, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.RADIANS, follower.getHeading());

        telemetry.addData("path state", pathState);
        telemetry.addLine("Position: " + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.update();

        if (pathState == -1)
        {
            requestOpModeStop();
        }
    }

    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
        goalPosition = Field.blueGoal;           // ← blue goal
        hw.turret.isTargeting = true;

        Field.lastAllianceSide = Field.Side.BLUE; // ← blue side
    }

    @Override
    public void init()
    {
        pathTimer = new Timer();
        shooterTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        hw = new HardwareManager(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();
        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.BLUE);   // ← blue color

        hw.intake.closeGate();

        shotsFired = false;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side currentSide = Field.Side.BLUE;                              // ← blue
    private Field.StartingPosition currentStartingPosition = Field.StartingPosition.NEAR;
    private int numOfSpikes = 3;

    @Override
    public void init_loop()
    {
        hw.lights.update();
        hw.turret.update();
        hw.intake.update();

        telemetry.addLine("Press up or down on the D-Pad to select number of spikes");
        telemetry.addLine("Number of spikes selected: " + numOfSpikes);
        if (gamepad1.dpadUpWasPressed() && numOfSpikes != 3)
        {
            numOfSpikes++;
        }
        else if (gamepad1.dpadDownWasPressed() && numOfSpikes != 0)
        {
            numOfSpikes--;
        }
        telemetry.addData("Current Location", follower.getPose());
        telemetry.addLine("Side: " + ((currentSide == Field.Side.BLUE) ? "Blue" : "Red"));
        telemetry.addLine("Starting Position: " + ((currentStartingPosition == Field.StartingPosition.NEAR) ? "Near" : "Far"));
    }
}