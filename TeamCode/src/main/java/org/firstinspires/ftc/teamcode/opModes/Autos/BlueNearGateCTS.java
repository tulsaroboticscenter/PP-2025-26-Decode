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

@Autonomous(name = "Blue Near Gate - CTS", group = "Autonomous", preselectTeleOp = "tele - CTS")
public class BlueNearGateCTS extends OpMode {

    // -------------------------------------------------------------------------
    // Tunable constants
    // -------------------------------------------------------------------------

    // Wait after arriving at scorePose for flywheel + turret to settle (ms)
    private static final long SHOT_SETTLE_MS  = 1000;

    // Gate stays open this long — all 3 artifacts feed through automatically (ms)
    private static final long SHOT_OPEN_MS    = 1500;

    // Safety timeout — move on if something jams during a shot (ms)
    private static final long SHOT_TIMEOUT_MS = 5000;

    // How long the robot dwells at clearGate waiting for 3 artifacts to fall
    // through the classifier after the gate opens. Tune during practice. (ms)
    private static final long GATE_DWELL_MS   = 3000;

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

    private int pathState = -1;
    private int gateCycle = 0;   // which of the 3 gate intake cycles we are on (1-3)

    public Pose2D goalPosition = null;

    // -------------------------------------------------------------------------
    // Field poses — same as BlueNearGateCTS
    // -------------------------------------------------------------------------

    private final Pose startPose        = new Pose(27.0873786407767, 133.11650485436894, Math.toRadians(-126.678));
    private final Pose scorePose        = new Pose(47,    84.1,  Math.toRadians(-45));
    private final Pose intake1          = new Pose(25,    70,    Math.toRadians(180));
    private final Pose clearGate        = new Pose(16,    67,    Math.toRadians(180));
    private final Pose gateControlPoint = new Pose(30,    61);
    private final Pose prepIntake2      = new Pose(42.8,  53,    Math.toRadians(180));
    private final Pose intake2          = new Pose(20,    49,    Math.toRadians(180));
    private final Pose park             = new Pose(29.058, 85.796, Math.toRadians(-90));

    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------

    // Preload
    private PathChain scorePreload;

    // Spike 1
    private PathChain driveToIntake1;   // scorePose → intake1
    private PathChain clearGatePath;    // intake1   → clearGate (Bezier curve)
    private PathChain scoreLine1;       // clearGate → scorePose

    // Gate cycles — built once, reused all 3 times
    private PathChain driveToGate;      // scorePose → clearGate (direct)
    private PathChain gateToScore;      // clearGate → scorePose

    // Spike 2
    private PathChain lineupIntake2;    // scorePose  → prepIntake2
    private PathChain intakeLine2;      // prepIntake2 → intake2
    private PathChain scoreLine2;       // intake2    → scorePose

    // Park (built dynamically from current pose)
    private PathChain parkPath;

    // -------------------------------------------------------------------------
    // Path building
    // -------------------------------------------------------------------------

    public void buildPaths() {

        // --- Preload ---------------------------------------------------------
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // --- Spike 1 ---------------------------------------------------------
        driveToIntake1 = follower.pathBuilder()
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

        // --- Gate cycles (reused 3 times) ------------------------------------
        // Robot drives directly from scorePose to clearGate, dwells while
        // artifacts fall through the classifier, then returns the same way.
        driveToGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, clearGate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(180))
                .build();

        gateToScore = follower.pathBuilder()
                .addPath(new BezierLine(clearGate, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                .build();

        // --- Spike 2 ---------------------------------------------------------
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

    // Shooter gate open window — all 3 artifacts feed through automatically
    private boolean shouldOpenGate() {
        long t = shooterTimer.getElapsedTime();
        return t >= SHOT_SETTLE_MS && t < SHOT_SETTLE_MS + SHOT_OPEN_MS;
    }

    // Shooter gate cycle complete — close and move on
    private boolean shotComplete() {
        return shooterTimer.getElapsedTime() >= SHOT_SETTLE_MS + SHOT_OPEN_MS;
    }

    // Safety net — don't hang if something jams
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
    //   0  → Drive startPose → scorePose
    //   1  → Fire preload (3 artifacts)
    //
    //   2  → Drive scorePose → intake1
    //   3  → Drive intake1  → clearGate (Bezier curve)
    //   4  → Drive clearGate → scorePose
    //   5  → Fire spike 1 (3 artifacts)
    //
    //   6  → Drive scorePose → clearGate (direct)
    //   7  → Dwell at clearGate for GATE_DWELL_MS, intake running
    //   8  → Drive clearGate → scorePose
    //   9  → Fire gate cycle batch (3 artifacts)
    //        gateCycle 1 → repeat states 6-9
    //        gateCycle 2 → repeat states 6-9
    //        gateCycle 3 → proceed to state 10
    //
    //  10  → Drive scorePose → prepIntake2
    //  11  → Drive prepIntake2 → intake2
    //  12  → Drive intake2 → scorePose
    //  13  → Fire spike 2 (3 artifacts)
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

            // --- Preload: fire 3 artifacts ------------------------------------
            case 1:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(driveToIntake1, true);
                    setPathState(2);
                }
                break;

            // --- Spike 1: drive to intake1 -----------------------------------
            case 2:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(clearGatePath, true);
                    setPathState(3);
                }
                break;

            // --- Spike 1: Bezier curve through gate --------------------------
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

            // --- Spike 1: fire 3 artifacts ------------------------------------
            case 5:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    // Begin first gate cycle
                    gateCycle = 1;
                    hw.intake.intake();
                    follower.followPath(driveToGate, true);
                    setPathState(6);
                }
                break;

            // --- Gate cycle: drive to clearGate ------------------------------
            case 6:
                if (!follower.isBusy()) {
                    // pathTimer resets automatically via setPathState
                    // so dwell starts counting from the moment we arrive
                    setPathState(7);
                }
                break;

            // --- Gate cycle: dwell at clearGate, intake running --------------
            // pathTimer was reset when entering this state.
            // Robot stays here for GATE_DWELL_MS while artifacts fall through
            // the classifier and are collected by the intake.
            case 7:
                hw.intake.intake();
                if (pathTimer.getElapsedTime() >= GATE_DWELL_MS) {
                    follower.followPath(gateToScore, true);
                    setPathState(8);
                }
                break;

            // --- Gate cycle: drive to score pose -----------------------------
            case 8:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(9);
                break;

            // --- Gate cycle: fire 3 artifacts ---------------------------------
            // Loop back for cycles 1 and 2. After cycle 3, go to spike 2.
            case 9:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
                } else if (shotComplete() || shotTimedOut()) {
                    hw.intake.closeGate();
                    hw.intake.stop();
                    if (gateCycle < 3) {
                        // More gate cycles remaining — return to gate
                        gateCycle++;
                        hw.intake.intake();
                        follower.followPath(driveToGate, true);
                        setPathState(6);
                    } else {
                        // All 3 gate cycles done — move to spike 2
                        follower.followPath(lineupIntake2, true);
                        setPathState(10);
                    }
                }
                break;

            // --- Spike 2: lineup to prepIntake2 ------------------------------
            case 10:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine2, true);
                    setPathState(11);
                }
                break;

            // --- Spike 2: drive to intake2 -----------------------------------
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLine2, true);
                    setPathState(12);
                }
                break;

            // --- Spike 2: drive to score pose --------------------------------
            case 12:
                ensureFlywheelSpinning();
                if (follower.isBusy()) { shooterTimer.resetTimer(); break; }
                setPathState(13);
                break;

            // --- Spike 2: fire 3 artifacts, then park ------------------------
            case 13:
                ensureFlywheelSpinning();
                if (shouldOpenGate()) {
                    hw.intake.openGate();
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

        telemetry.addLine("Alliance: Blue  |  Position: Near Gate");
        telemetry.addLine("Sequence: Preload → Spike 1 → Gate x3 → Spike 2 → Park");
        telemetry.addLine("Max artifacts: 18  (3 preload + 3 spike1 + 9 gate + 3 spike2)");
        telemetry.addLine("Gate dwell: " + (GATE_DWELL_MS / 1000.0) + "s per cycle");
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

        telemetry.addData("Path state",      pathState);
        telemetry.addData("Gate cycle",      gateCycle + " / 3");
        telemetry.addData("Shooter timer",   shooterTimer.getElapsedTime() + " ms");
        telemetry.addData("Gate dwell",      pathState == 7
                ? pathTimer.getElapsedTime() + " / " + GATE_DWELL_MS + " ms"
                : "—");
        telemetry.addData("Shot gate",       shouldOpenGate() ? "OPEN" : "closed");
        telemetry.addData("Flywheel target", hw.turret.velocity);
        telemetry.addData("Flywheel actual", hw.turret.launcherL.getVelocity());
        telemetry.addData("Distance",        hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.addLine("Position: "       + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Match time",      String.format("%.1f", opmodeTimer.getElapsedTimeSeconds()) + "s");
        telemetry.update();

        if (pathState == -1) requestOpModeStop();
    }
}