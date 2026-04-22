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

@Autonomous(name = "Blue Near - CTS", group = "Autonomous", preselectTeleOp = "tele - CTS")
public class BlueNearCTS extends OpMode {

    // -------------------------------------------------------------------------
    // Tunable constants
    // -------------------------------------------------------------------------

    // Wait after arriving at scorePose for flywheel + turret to settle (ms)
    private static final long SHOT_SETTLE_MS    = 1000;

    // Gate stays open this long — all 3 artifacts feed through automatically (ms)
    private static final long SHOT_OPEN_MS      = 1500;

    // Safety timeout — move on if something jams (ms)
    private static final long SHOT_TIMEOUT_MS   = 5000;

    // Maximum time allowed on any intake path before moving on (ms)
    private static final long INTAKE_TIMEOUT_MS = 4000;

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    private Follower follower;
    private final HardwareManager hw = new HardwareManager(hardwareMap);

    // -------------------------------------------------------------------------
    // Timers
    // -------------------------------------------------------------------------

    private Timer pathTimer    = new Timer();
    private Timer shooterTimer = new Timer();
    private Timer opmodeTimer  = new Timer();

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    private int pathState    = -1;
    private int numOfSpikes  = 3;   // configurable in init_loop (0-3)
    private int currentSpike = 0;   // for telemetry

    public Pose2D goalPosition = null;

    // -------------------------------------------------------------------------
    // Field poses
    // -------------------------------------------------------------------------

    private final Pose startPose        = new Pose(27.0873786407767, 133.11650485436894, Math.toRadians(-126.678));
    private final Pose scorePose        = new Pose(47,    84.1,  Math.toRadians(-45));
    private final Pose intake1          = new Pose(25,    70,    Math.toRadians(180));
    private final Pose clearGate        = new Pose(16,    67,    Math.toRadians(180));
    private final Pose gateControlPoint = new Pose(30,    61);
    private final Pose prepIntake2      = new Pose(42.8,  53,    Math.toRadians(180));
    private final Pose intake2          = new Pose(20,    49,    Math.toRadians(180));
    private final Pose prepIntake3      = new Pose(45,    28,    Math.toRadians(180));
    private final Pose intake3          = new Pose(20,    28,    Math.toRadians(180));
    private final Pose park             = new Pose(29.058, 85.796, Math.toRadians(-90));

    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------

    private PathChain scorePreload;
    private PathChain intakeLine1, clearGatePath, scoreLine1;
    private PathChain lineupIntake2, intakeLine2, scoreLine2;
    private PathChain lineupIntake3, intakeLine3, scoreLine3;
    private PathChain parkPath;

    // -------------------------------------------------------------------------
    // Path building
    // -------------------------------------------------------------------------

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Spike 1
        intakeLine1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        clearGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(intake1, gateControlPoint, clearGate))
                .setConstantHeadingInterpolation(clearGate.getHeading())
                .build();
        scoreLine1 = follower.pathBuilder()
                .addPath(new BezierLine(clearGate, scorePose))
                .setLinearHeadingInterpolation(clearGate.getHeading(), Math.toRadians(-45))
                .build();

        // Spike 2
        lineupIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepIntake2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .build();
        intakeLine2 = follower.pathBuilder()
                .addPath(new BezierLine(prepIntake2, intake2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scoreLine2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, scorePose))
                .setLinearHeadingInterpolation(intake2.getHeading(), Math.toRadians(-45))
                .build();

        // Spike 3
        lineupIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepIntake3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .build();
        intakeLine3 = follower.pathBuilder()
                .addPath(new BezierLine(prepIntake3, intake3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scoreLine3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, scorePose))
                .setLinearHeadingInterpolation(intake3.getHeading(), Math.toRadians(-45))
                .build();
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private void startPark() {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), park))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
        follower.followPath(parkPath, true);
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    // Gate stays open while all 3 artifacts feed through
    private boolean shouldOpenGate() {
        long t = shooterTimer.getElapsedTime();
        return t >= SHOT_SETTLE_MS && t < SHOT_SETTLE_MS + SHOT_OPEN_MS;
    }

    // All 3 have fed through — close gate and move on
    private boolean shotComplete() {
        return shooterTimer.getElapsedTime() >= SHOT_SETTLE_MS + SHOT_OPEN_MS;
    }

    // Safety — don't hang forever if something jams
    private boolean shotTimedOut() {
        return shooterTimer.getElapsedTime() >= SHOT_TIMEOUT_MS;
    }

    // Keep flywheel at speed throughout the match
    private void ensureFlywheelSpinning() {
        if (!hw.turret.isFlywheelSpinning) hw.turret.spinUpFlywheel();
    }

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    //
    //  Each scoring run fires all 3 artifacts with a single gate open:
    //
    //   0  → Drive startPose  → scorePose  (preload: 3 artifacts already loaded)
    //   1  → Fire 3 preloaded artifacts
    //
    //   2  → Drive scorePose  → intake1
    //   3  → Drive clearGatePath (curve around gate obstacle)
    //   4  → Drive clearGate  → scorePose
    //   5  → Fire 3 artifacts from spike 1
    //
    //   6  → Drive scorePose  → prepIntake2
    //   7  → Drive prepIntake2 → intake2
    //   8  → Drive intake2    → scorePose
    //   9  → Fire 3 artifacts from spike 2
    //
    //  10  → Drive scorePose  → prepIntake3
    //  11  → Drive prepIntake3 → intake3
    //  12  → Drive intake3    → scorePose
    //  13  → Fire 3 artifacts from spike 3
    //
    //  14  → Park
    //  -1  → Done
    //
    // -------------------------------------------------------------------------

    private void updateStateMachine() {
        switch (pathState) {

            // --- Preload: drive to score pose --------------------------------
            case 0:
                hw.turret.isTargeting = true;
                hw.turret.spinUpFlywheel();
                hw.intake.closeGate();
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            // --- Preload: fire all 3 artifacts (single gate open) ------------
            case 1:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                    hw.intake.forceIntake();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    if (numOfSpikes >= 1) {
                        hw.intake.intake();
                        follower.followPath(intakeLine1, true);
                        currentSpike = 1;
                        setPathState(2);
                    } else {
                        startPark();
                        setPathState(14);
                    }
                }
                break;

            // --- Spike 1: drive to intake1 -----------------------------------
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > INTAKE_TIMEOUT_MS) {
                    hw.intake.intake();
                    follower.followPath(clearGatePath, true);
                    setPathState(3);
                }
                break;

            // --- Spike 1: clear gate curve -----------------------------------
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLine1, true);
                    setPathState(4);
                }
                break;

            // --- Spike 1: drive to score pose --------------------------------
            case 4:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(5);
                break;

            // --- Spike 1: fire all 3 artifacts (single gate open) ------------
            case 5:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                    hw.intake.forceIntake();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    if (numOfSpikes >= 2) {
                        follower.followPath(lineupIntake2, true);
                        currentSpike = 2;
                        setPathState(6);
                    } else {
                        startPark();
                        setPathState(14);
                    }
                }
                break;

            // --- Spike 2: lineup to prepIntake2 ------------------------------
            case 6:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine2, true);
                    setPathState(7);
                }
                break;

            // --- Spike 2: drive to intake2 -----------------------------------
            case 7:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > INTAKE_TIMEOUT_MS) {
                    follower.followPath(scoreLine2, true);
                    setPathState(8);
                }
                break;

            // --- Spike 2: drive to score pose --------------------------------
            case 8:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(9);
                break;

            // --- Spike 2: fire all 3 artifacts (single gate open) ------------
            case 9:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                    hw.intake.forceIntake();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    if (numOfSpikes >= 3) {
                        follower.followPath(lineupIntake3, true);
                        currentSpike = 3;
                        setPathState(10);
                    } else {
                        startPark();
                        setPathState(14);
                    }
                }
                break;

            // --- Spike 3: lineup to prepIntake3 ------------------------------
            case 10:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine3, true);
                    setPathState(11);
                }
                break;

            // --- Spike 3: drive to intake3 -----------------------------------
            case 11:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > INTAKE_TIMEOUT_MS) {
                    follower.followPath(scoreLine3, true);
                    setPathState(12);
                }
                break;

            // --- Spike 3: drive to score pose --------------------------------
            case 12:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(13);
                break;

            // --- Spike 3: fire all 3 artifacts, then park --------------------
            case 13:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                    hw.intake.forceIntake();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    startPark();
                    setPathState(14);
                }
                break;

            // --- Park --------------------------------------------------------
            case 14:
                hw.intake.closeGate();
                if (!follower.isBusy()) setPathState(-1);
                break;
        }
    }

    // -------------------------------------------------------------------------
    // OpMode lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void init() {
        pathTimer    = new Timer();
        shooterTimer = new Timer();
        opmodeTimer  = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.BLUE);
        hw.intake.closeGate();

        buildPaths();

        telemetry.addData("Path state", pathState);
        telemetry.addData("x",          follower.getPose().getX());
        telemetry.addData("y",          follower.getPose().getY());
        telemetry.addData("Heading",    Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        hw.lights.update();
        hw.turret.update();

        if (gamepad1.dpadUpWasPressed()   && numOfSpikes < 3) numOfSpikes++;
        if (gamepad1.dpadDownWasPressed() && numOfSpikes > 0) numOfSpikes--;

        int totalArtifacts = (numOfSpikes * 3) + 3;
        telemetry.addLine("dpad up/down to set spike count");
        telemetry.addLine("Spikes: " + numOfSpikes
                + "  →  " + totalArtifacts + " total artifacts");
        telemetry.addLine("Alliance: Blue  |  Position: Near Gate");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shooterTimer.resetTimer();

        goalPosition = Field.blueGoal;
        Field.lastAllianceSide = Field.Side.BLUE;

        hw.turret.isTargeting = true;

        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // Turret tracks goal continuously — isTargeting is true from start()
        hw.turret.setTarget(follower.getPose(), goalPosition);
        hw.turret.updateFlywheelAndHood(
                new Pose2D(DistanceUnit.INCH,
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        AngleUnit.RADIANS,
                        follower.getPose().getHeading()),
                goalPosition);

        hw.updatePedro();

        updateStateMachine();

        // Persist position for TeleOp handoff
        Field.lastKnownPosition = new Pose2D(
                DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.RADIANS,
                follower.getHeading());

        telemetry.addData("Path state",      pathState);
        telemetry.addData("Spike",           currentSpike + " / " + numOfSpikes);
        telemetry.addData("Shooter timer",   shooterTimer.getElapsedTime() + " ms");
        telemetry.addData("Gate",            shouldOpenGate() ? "OPEN" : "closed");
        telemetry.addData("Flywheel target", hw.turret.targetVelocity);
        telemetry.addData("Flywheel actual", hw.turret.launcherL.getVelocity());
        telemetry.addData("Distance",        hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.addLine("Position: "       + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Match time",      String.format("%.1f", opmodeTimer.getElapsedTimeSeconds()) + "s");
        telemetry.update();

        if (pathState == -1) requestOpModeStop();
    }
}