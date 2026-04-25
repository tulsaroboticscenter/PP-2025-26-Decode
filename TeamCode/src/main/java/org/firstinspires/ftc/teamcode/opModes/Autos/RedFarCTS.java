package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Red Far Gate", group = "Autonomous", preselectTeleOp = "TeleOp")
public class RedFarCTS extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, shooterTimer;

    private HardwareManager hw;

    private int pathState;

    public Pose2D goalPosition = null;

    // --- Poses ---

    // Starting position (from redFar)
    private final Pose startPose = Field.toPedro(Field.redSmallZone);

    // Shooting position — robot stays at start and shoots in place
    private final Pose shootPose = startPose;

    // Spike mark 3 intake position (from redFar's intakeCurve3 endpoint)
    private final Pose spike3Intake = new Pose(129.989, 83.870, Math.toRadians(0));

    // Human player zone position (from RedNearGate's intake1, single fixed position)
    private final Pose humanPlayerIntake = new Pose(120.7, 80, Math.toRadians(0));

    // Scoring/shooting pose (from RedNearGate's scorePose — after returning from field side)
    private final Pose scorePose = new Pose(90, 77, Math.toRadians(225));

    // Park pose (from redFar)
    private final Pose park = new Pose(108, 15.5, Math.toRadians(90));

    // --- Paths ---
    private PathChain parkPath;
    private PathChain toSpike3, fromSpike3;
    private PathChain toHumanPlayer, fromHumanPlayer;

    public void buildPaths() {
        // Straight line from start/shoot pose to spike mark 3
        toSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, spike3Intake))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Straight line back from spike 3 to scoring pose
        fromSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3Intake, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        // Straight line from scoring pose to human player zone
        toHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, humanPlayerIntake))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Straight line from human player zone back to scoring pose
        fromHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(humanPlayerIntake, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();
    }

    public void Park() {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
        follower.followPath(parkPath);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // -------------------------------------------------------
            // PHASE 1: Spin up and shoot preloaded artifacts in place
            // -------------------------------------------------------

            // Spin up flywheel, begin shooting timer
            case 0:
                hw.turret.spinUpFlywheel();
                shooterTimer.resetTimer();
                setPathState(1);
                break;

            // Wait for flywheel spin-up, then feed and shoot preloads
            case 1:
                if (shooterTimer.getElapsedTime() > 3000) {
                    hw.intake.partialIntake();
                    hw.intake.openGate();
                }
                if (shooterTimer.getElapsedTime() > 6000) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    // Head to spike mark 3
                    follower.followPath(toSpike3, true);
                    setPathState(2);
                }
                break;

            // -------------------------------------------------------
            // PHASE 2: Spike mark 3 — intake and return to shoot
            // -------------------------------------------------------

            // Drive to spike mark 3, intaking along the way
            case 2:
                if (!follower.isBusy()) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(fromSpike3, true);
                    shooterTimer.resetTimer();
                    setPathState(3);
                }
                break;

            // Arrived at score pose — shoot spike 3 artifacts
            case 3:
                if (follower.isBusy()) {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy() && shooterTimer.getElapsedTime() > 1500) {
                    hw.intake.partialIntake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000) {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        // Head into human player zone
                        follower.followPath(toHumanPlayer, true);
                        shooterTimer.resetTimer();
                        setPathState(4);
                    }
                }
                break;

            // -------------------------------------------------------
            // PHASE 3: Human player zone cycle (repeats until time)
            // -------------------------------------------------------

            // Drive to human player zone, intaking
            case 4:
                if (!follower.isBusy()) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(fromHumanPlayer, true);
                    shooterTimer.resetTimer();
                    setPathState(5);
                }
                break;

            // Drive back to score pose and shoot
            case 5:
                if (follower.isBusy()) {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy() && shooterTimer.getElapsedTime() > 1500) {
                    hw.intake.partialIntake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000) {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        // Loop back to human player zone
                        follower.followPath(toHumanPlayer, true);
                        shooterTimer.resetTimer();
                        setPathState(4);
                    }
                }
                break;

            // -------------------------------------------------------
            // PARK (transitioned to externally by time limit / driver)
            // -------------------------------------------------------

            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                    stop();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        hw.lights.update();
        hw.turret.update();
        hw.intake.update();

        hw.turret.setTarget(follower.getPose(), goalPosition);

        // Constantly save the last known position
        Field.lastKnownPosition = new Pose2D(
                DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.RADIANS,
                follower.getHeading()
        );

        // Auto-park with ~3 seconds remaining (adjust threshold as needed)
        if (opmodeTimer.getElapsedTime() > 27000 && pathState != 8 && pathState != -1) {
            Park();
            setPathState(8);
        }

        telemetry.addLine("RED FAR GATE");
        telemetry.addData("path state", pathState);
        telemetry.addLine("Position: " + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Elapsed Time (ms)", opmodeTimer.getElapsedTime());
        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.update();

        if (pathState == -1) {
            requestOpModeStop();
        }
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        goalPosition = Field.redGoal;
        Field.lastAllianceSide = Field.Side.RED;
    }

    @Override
    public void init() {
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
        hw.lights.setLightColor(RGBLightController.RED);
        hw.intake.closeGate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        hw.lights.update();
        hw.turret.update();

        telemetry.addLine("RED FAR GATE — Ready");
        telemetry.addData("Current Pose", follower.getPose());
        telemetry.update();
    }
}