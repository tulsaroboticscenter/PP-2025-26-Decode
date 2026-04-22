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

@Autonomous(name = "Blue Far - CTS", group = "Autonomous", preselectTeleOp = "tele - CTS")
public class BlueFarCTS extends OpMode {

    // -------------------------------------------------------------------------
    // Tunable constants
    // -------------------------------------------------------------------------

    // Wait after arriving at scorePose for flywheel + turret to settle (ms)
    private static final long SHOT_SETTLE_MS = 1000;

    // Gate stays open this long — all 3 artifacts feed through automatically (ms)
    private static final long SHOT_OPEN_MS = 1500;

    // Safety timeout — move on if something jams (ms)
    private static final long SHOT_TIMEOUT_MS = 5000;

    // Maximum time allowed on any intake path before moving on (ms)
    private static final long INTAKE_TIMEOUT_MS = 4000;

    // How long one full human player cycle takes (drive in + intake + reverse + score).
    // If less than this many seconds remain in the match, skip the next cycle and park.
    // Tune this after your first practice run tonight.
    private static final double HP_CYCLE_TIME_SECONDS = 10.0;

    // FTC autonomous period length (seconds)
    private static final double AUTO_DURATION_SECONDS = 30.0;

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
    private int hpCycleCount = 0;   // how many human player cycles completed

    public Pose2D goalPosition = null;

    // -------------------------------------------------------------------------
    // Field poses
    // -------------------------------------------------------------------------

    // Starting pose — back wall, blue side (from blueFar)
    private final Pose startPose = new Pose(60, 9, Math.toRadians(90));

    // Fixed score pose — derived from scoreLine endpoints in blueFar
    private final Pose scorePose = new Pose(56.311, 16.485, Math.toRadians(90));

    // Spike 3 intake end pose — the back-wall-closest spike (from intakeCurve3 in blueFar)
    private final Pose spike3ControlA  = new Pose(48.801, 83.680);
    private final Pose spike3ControlB  = new Pose(41.947, 84.748);
    private final Pose spike3IntakePose = new Pose(19.000, 84.010, Math.toRadians(180));

    // Human player zone staging pose — robot reverses straight back to here after intaking
    // Offset in front of the actual HP zone so the robot has a clean reverse exit path.
    // Tune x/y after first practice run.
    private final Pose hpStagingPose = new Pose(56.311, 16.485, Math.toRadians(180));

    // Park pose (from blueFar)
    private final Pose park = new Pose(36, 10, Math.toRadians(90));

    // -------------------------------------------------------------------------
    // Paths — built once, reused every cycle
    // -------------------------------------------------------------------------

    private PathChain driveToScore;       // startPose → scorePose
    private PathChain driveToSpike3;      // scorePose → spike3IntakePose (Bezier)
    private PathChain spike3ToScore;      // spike3IntakePose → scorePose
    private PathChain driveToHP;          // scorePose → Field.blueHumanPlayerZone
    private PathChain hpReverse;          // Field.blueHumanPlayerZone → hpStagingPose (reverse)
    private PathChain stagingToScore;     // hpStagingPose → scorePose
    private PathChain parkPath;           // current pose → park (built dynamically)

    // -------------------------------------------------------------------------
    // Path building
    // -------------------------------------------------------------------------

    public void buildPaths() {

        // Preload: straight line from start to score pose
        driveToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Spike 3: Bezier curve from score pose sweeping up to spike 3
        // Uses same control points as intakeCurve3 in blueFar
        driveToSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        spike3ControlA,
                        spike3ControlB,
                        spike3IntakePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Spike 3 return: straight line back to score pose, reversed
        spike3ToScore = follower.pathBuilder()
                .addPath(new BezierLine(spike3IntakePose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Human player zone approach: score pose → HP zone, heading 180° (intake faces HP wall)
        driveToHP = follower.pathBuilder()
                .addPath(new BezierLine(
                        scorePose,
                        new Pose(
                                Field.blueHumanPlayerZone.getX(DistanceUnit.INCH),
                                Field.blueHumanPlayerZone.getY(DistanceUnit.INCH),
                                Math.toRadians(180))))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .build();

        // Human player zone exit: reverse straight back out to staging pose
        // setReversed() makes Pedro drive backwards along the path
        hpReverse = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(
                                Field.blueHumanPlayerZone.getX(DistanceUnit.INCH),
                                Field.blueHumanPlayerZone.getY(DistanceUnit.INCH),
                                Math.toRadians(180)),
                        hpStagingPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setReversed()
                .build();

        // Staging to score: drive forward from staging pose to score pose
        stagingToScore = follower.pathBuilder()
                .addPath(new BezierLine(hpStagingPose, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), scorePose.getHeading())
                .build();
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private void startPark() {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();
        follower.followPath(parkPath, true);
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    // Gate open window — all 3 artifacts feed through automatically
    private boolean shouldOpenGate() {
        long t = shooterTimer.getElapsedTime();
        return t >= SHOT_SETTLE_MS && t < SHOT_SETTLE_MS + SHOT_OPEN_MS;
    }

    // Gate cycle complete — close and move on
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

    // Returns true if there is enough match time left to complete another full HP cycle
    private boolean timeForAnotherCycle() {
        double elapsed = opmodeTimer.getElapsedTimeSeconds();
        return (AUTO_DURATION_SECONDS - elapsed) >= HP_CYCLE_TIME_SECONDS;
    }

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    //
    //  0  → Drive startPose → scorePose
    //  1  → Fire 3 preloaded artifacts
    //
    //  2  → Drive scorePose → spike3IntakePose (Bezier)
    //  3  → Drive spike3IntakePose → scorePose (reversed)
    //  4  → Fire 3 artifacts from spike 3
    //
    //  5  → Drive scorePose → blueHumanPlayerZone  (intake running)
    //  6  → Reverse straight out → hpStagingPose
    //  7  → Drive hpStagingPose → scorePose
    //  8  → Fire 3 artifacts
    //       → if time remains: loop back to state 5
    //       → if no time: go to state 9
    //
    //  9  → Park
    //  -1 → Done
    //
    // -------------------------------------------------------------------------

    private void updateStateMachine() {
        switch (pathState) {

            // --- Preload: drive to score pose --------------------------------
            case 0:
                hw.turret.isTargeting = true;
                hw.turret.spinUpFlywheel();
                hw.intake.closeGate();
                follower.followPath(driveToScore, true);
                setPathState(1);
                break;

            // --- Preload: fire all 3 artifacts --------------------------------
            case 1:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(driveToSpike3, true);
                    setPathState(2);
                }
                break;

            // --- Spike 3: drive to intake pose (Bezier) ----------------------
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > INTAKE_TIMEOUT_MS) {
                    follower.followPath(spike3ToScore, true);
                    setPathState(3);
                }
                break;

            // --- Spike 3: reverse back to score pose -------------------------
            case 3:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(4);
                break;

            // --- Spike 3: fire all 3 artifacts --------------------------------
            case 4:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    // Check time before starting first HP cycle
                    if (timeForAnotherCycle()) {
                        hw.intake.intake();
                        follower.followPath(driveToHP, true);
                        setPathState(5);
                    } else {
                        startPark();
                        setPathState(9);
                    }
                }
                break;

            // --- HP cycle: drive into human player zone, intake running ------
            case 5:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > INTAKE_TIMEOUT_MS) {
                    // Robot is now at HP zone — reverse straight back out
                    follower.followPath(hpReverse, true);
                    setPathState(6);
                }
                break;

            // --- HP cycle: reverse out of human player zone ------------------
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(stagingToScore, true);
                    setPathState(7);
                }
                break;

            // --- HP cycle: drive to score pose --------------------------------
            case 7:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(8);
                break;

            // --- HP cycle: fire all 3 artifacts, loop or park ----------------
            case 8:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    hpCycleCount++;

                    if (timeForAnotherCycle()) {
                        // Enough time — run another HP cycle
                        hw.intake.intake();
                        follower.followPath(driveToHP, true);
                        setPathState(5);
                    } else {
                        // Not enough time — park
                        startPark();
                        setPathState(9);
                    }
                }
                break;

            // --- Park --------------------------------------------------------
            case 9:
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

        telemetry.addLine("Alliance: Blue  |  Position: Back Wall");
        telemetry.addLine("Sequence: Preload → Spike 3 → HP zone (repeating)");
        telemetry.addLine("HP cycle budget: " + HP_CYCLE_TIME_SECONDS + "s per cycle");
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

        // Turret tracks goal continuously every loop
        hw.turret.setTarget(follower.getPose(), goalPosition);
        hw.turret.updateFlywheelAndHood(
                new Pose2D(DistanceUnit.INCH,
                        follower.getPose().getX(),
                        follower.getPose().getY(),
                        AngleUnit.RADIANS,
                        follower.getPose().getHeading()),
                goalPosition);

        hw.turret.update();
        hw.lights.update();

        updateStateMachine();

        // Persist position for TeleOp handoff
        Field.lastKnownPosition = new Pose2D(
                DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.RADIANS,
                follower.getHeading());

        // Telemetry
        double timeLeft = AUTO_DURATION_SECONDS - opmodeTimer.getElapsedTimeSeconds();
        telemetry.addData("Path state",      pathState);
        telemetry.addData("HP cycles done",  hpCycleCount);
        telemetry.addData("Time left",       String.format("%.1f", timeLeft) + "s");
        telemetry.addData("Cycle fits",      timeForAnotherCycle() ? "YES" : "no");
        telemetry.addData("Shooter timer",   shooterTimer.getElapsedTime() + " ms");
        telemetry.addData("Gate",            shouldOpenGate() ? "OPEN" : "closed");
        telemetry.addData("Flywheel target", hw.turret.targetVelocity);
        telemetry.addData("Flywheel actual", hw.turret.launcherL.getVelocity());
        telemetry.addData("Distance",        hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.addLine("Position: "       + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.update();

        if (pathState == -1) requestOpModeStop();
    }
}