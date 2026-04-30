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

@Autonomous(name = "Blue Far Leviathan CTS", group = "Autonomous", preselectTeleOp = "tele")
public class BlueFarCTS extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HardwareManager hw;
    private Timer shooterTimer;
    private int pathState;
    public boolean shotsFired;
    public Pose2D goalPosition = null;

    // ── Poses ────────────────────────────────────────────────────────────────
    private final Pose scorePose      = new Pose(51, 10, Math.toRadians(180));
    private final Pose startPose      = Field.toPedro(Field.blueSmallZone);
    private final Pose park           = new Pose(30, 10, Math.toRadians(180));
    private final Pose intakePose = new Pose(6,37, Math.toRadians(180));

    // HPZ intake end-point (where the robot sits while waiting for a ring)
    private final Pose hpzPose        = new Pose(6, 10, Math.toRadians(180));

    // ── Paths ────────────────────────────────────────────────────────────────
    /** Spike-mark sweep: arcs out to collect the 3 pre-loaded rings */
    public PathChain moveToSpikeMark;

    /** Return from spike mark to shoot */
    public PathChain spikeMarkToShoot;

    /**
     * Shoot → HPZ.
     * Starts at scorePose, ends at hpzPose.
     * Reused every HPZ cycle.
     */
    public PathChain shootToHPZ;

    /**
     * HPZ → Shoot.
     * Starts at hpzPose, ends at scorePose.
     * Reused every HPZ cycle.
     */
    public PathChain hpzToShoot;

    /** Final park path */
    public PathChain parkPath;

    // ── Timing constants (ms) ────────────────────────────────────────────────
    private static final long PRELOAD_GATE_OPEN_MS  = 1000;
    private static final long PRELOAD_SHOOT_MS       = 2500;
    private static final long SHOOT_SETTLE_MS        = 750;
    private static final long SHOOT_DURATION_MS      = 1750;
    private static final long HPZ_COLLECT_MS         = 1500;
    private static final long TIME_LIMIT_MS          = 27_000;

    public void buildPaths() {

        // Spike mark sweep (mirrored across Y = 72)
        moveToSpikeMark = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(43, 35), intakePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Spike mark → score
        spikeMarkToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(8, 15), scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Score → HPZ (gentle curve to avoid field walls)
        shootToHPZ = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(10, 8),
                        hpzPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // HPZ → Score
        hpzToShoot = follower.pathBuilder()
                .addPath(new BezierLine(hpzPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Park
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, park))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    // ── State machine ────────────────────────────────────────────────────────

    public void autonomousPathBlueUpdate() {
        if (opmodeTimer.getElapsedTime() > 28_000 && pathState != 20 && pathState != -1) {
            follower.followPath(parkPath, true);
            setPathState(20);
            return;
        }
        switch (pathState) {

            // ── STATE 0: Spin up flywheel, open gate for preload shot ─────────
            case 0:
                hw.turret.spinUpFlywheel();
                hw.intake.openGate();
                hw.turret.setTarget(follower.getPose(), goalPosition);
                shooterTimer.resetTimer();
                setPathState(1);
                break;

            // ── STATE 1: Shoot preloads ───────────────────────────────────────
            case 1:
                if (follower.isBusy()) {
                    hw.intake.intake();

                    shooterTimer.resetTimer();
                    break;
                }
                if (shooterTimer.getElapsedTime() > PRELOAD_GATE_OPEN_MS) {
                    hw.intake.openGate();
                    hw.intake.intake();
                }
                if (shooterTimer.getElapsedTime() > PRELOAD_SHOOT_MS) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(moveToSpikeMark, true);
                    setPathState(2);
                }
                break;

            // ── STATE 2: Travel to / collect spike-mark rings ─────────────────
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > HPZ_COLLECT_MS) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(spikeMarkToShoot, true);
                    hw.intake.partialIntake();
                    setPathState(3);
                }
                break;

            // ── STATE 3: Shoot spike-mark rings, then head to HPZ ────────────
            case 3:
                if (follower.isBusy()) {
                    hw.intake.partialIntake();

                    shooterTimer.resetTimer();
                    break;
                }
                if (shooterTimer.getElapsedTime() > SHOOT_SETTLE_MS) {
                    hw.intake.openGate();
                    hw.intake.partialIntake();
                }
                if (shooterTimer.getElapsedTime() > SHOOT_DURATION_MS) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(shootToHPZ, true);
                    setPathState(10);
                }
                break;

            // ── STATE 10: Wait at HPZ, collect ring, head back ────────────────
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > HPZ_COLLECT_MS) {
                    hw.intake.intake();
                    follower.followPath(hpzToShoot, true);
                    hw.intake.partialIntake();
                    shooterTimer.resetTimer();
                    setPathState(11);
                }
                break;

            // ── STATE 11: Arrive at score, shoot, decide next action ──────────
            case 11:
                if (follower.isBusy()) {
                    hw.intake.partialIntake();

                    shooterTimer.resetTimer();
                    break;
                }
                if (shooterTimer.getElapsedTime() > SHOOT_SETTLE_MS) {
                    hw.intake.openGate();
                    hw.intake.partialIntake();

                }
                if (shooterTimer.getElapsedTime() > SHOOT_DURATION_MS) {
                    hw.intake.closeGate();
                    hw.intake.intake();

                    if (opmodeTimer.getElapsedTime() < TIME_LIMIT_MS) {
                        follower.followPath(shootToHPZ, true);
                        setPathState(10);
                    } else {
                        follower.followPath(parkPath, true);
                        setPathState(20);
                    }
                }
                break;

            // ── STATE 20: Park ────────────────────────────────────────────────
            case 20:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // ── OpMode lifecycle ─────────────────────────────────────────────────────

    @Override
    public void loop() {
        follower.update();
        autonomousPathBlueUpdate();

        hw.lights.update();
        hw.turret.update();
        hw.intake.update();

        //hw.turret.setTarget(follower.getPose(), goalPosition);
        hw.turret.updateFlywheelAndHood(follower.getPose(), goalPosition);

        Field.lastKnownPosition = new Pose2D(
                DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.RADIANS,
                follower.getHeading());

        telemetry.addLine("BLUE Far Leviathan");
        telemetry.addData("Path State",    pathState);
        telemetry.addData("OpMode Time",   opmodeTimer.getElapsedTime() + " ms");
        telemetry.addData("Shooter Timer", shooterTimer.getElapsedTime() + " ms");
        telemetry.addData("Position",      PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Dist to Goal",  hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.update();

        if (pathState == -1) requestOpModeStop();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        goalPosition = Field.blueGoal;
        hw.turret.isTargeting = true;
        Field.lastAllianceSide = Field.Side.BLUE;
    }

    @Override
    public void init() {
        pathTimer    = new Timer();
        shooterTimer = new Timer();
        opmodeTimer  = new Timer();
        opmodeTimer.resetTimer();

        hw       = new HardwareManager(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.BLUE);
        hw.intake.closeGate();
        shotsFired = false;

        telemetry.addData("Path State", pathState);
        telemetry.addData("x",          follower.getPose().getX());
        telemetry.addData("y",          follower.getPose().getY());
        telemetry.addData("Heading",    Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side             currentSide             = Field.Side.BLUE;
    private Field.StartingPosition currentStartingPosition = Field.StartingPosition.FAR;

    @Override
    public void init_loop() {
        hw.lights.update();
        hw.turret.update();
        hw.intake.update();

        telemetry.addLine("Side: Blue");
        telemetry.addLine("Starting Position: Far");
    }
}