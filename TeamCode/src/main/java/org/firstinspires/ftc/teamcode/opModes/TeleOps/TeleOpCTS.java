package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "tele - CTS", group = "Robot")
public class TeleOpCTS extends OpMode {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    // Robot begins slewing turret + ramping flywheel when closer than this (inches)
    private static final double PRE_AIM_DISTANCE_INCHES = 80.0;

    // Flywheel is considered "ready to fire" when within this many ticks/s of target
    private static final double FLYWHEEL_READY_TOLERANCE = 75.0;

    // How long the "ready" rumble pattern lasts (ms) — prevents repeat spam
    private static final double READY_RUMBLE_COOLDOWN_MS = 2000.0;

    // -------------------------------------------------------------------------
    // Hardware + state
    // -------------------------------------------------------------------------

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private HardwareManager hw;
    private Pose2D goalPosition = null;

    // Only one runtime needed — endgame checks and ready-rumble cooldown
    private final ElapsedTime totalRuntime      = new ElapsedTime();
    private final ElapsedTime readyRumbleTimer  = new ElapsedTime();

    // Position / heading
    private Pose2D storedLocation       = null;
    private Field.Side startingSide     = null;
    private boolean loaded              = false;

    // Parking state machine
    private boolean isParking = false;

    private enum ParkStatus { NOT_PARKED, MOBILE_PARKED, FULL_PARKED }
    private ParkStatus parkStatus = ParkStatus.NOT_PARKED;

    // Endgame + flywheel-ready flags
    private boolean endgame         = false;
    private boolean flywheelWasReady = false;

    // Init-loop testing overrides
    private boolean testing         = false;
    private Field.Side testingSide  = Field.Side.RED;

    // Cached each loop — read every motor value once and reuse everywhere
    private Pose2D pos;
    private double distance;
    private double leftVelocity, rightVelocity, avgVelocity;
    private double leftFlyCurrent, rightFlyCurrent;
    private double driveCurrentTotal, intakeCurrent, turretCurrentTotal, totalRobotCurrent;

    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------

    @Override
    public void init() {
        hw = new HardwareManager(hardwareMap);  // initialize here
        hw.initTeleOp(hardwareMap);

        if (Field.lastKnownPosition != null) {
            startingSide = Field.lastAllianceSide;

            if (startingSide == Field.Side.BLUE) {
                hw.lights.setLightColor(RGBLightController.BLUE);
                goalPosition = Field.blueGoal;
                telemetry.addLine("Blue side.");
            } else if (startingSide == Field.Side.RED) {
                hw.lights.setLightColor(RGBLightController.RED);
                goalPosition = Field.redGoal;
                telemetry.addLine("Red side.");
            } else {
                // Defensive fallback
                startingSide = Field.Side.RED;
                hw.lights.setLightColor(RGBLightController.RED);
                goalPosition = Field.redGoal;
                telemetry.addLine("Alliance side not found. Defaulting to Red.");
            }

            storedLocation = Field.lastKnownPosition;
            loaded = true;
            telemetry.addLine("Position found: "
                    + storedLocation.getX(DistanceUnit.INCH) + ", "
                    + storedLocation.getY(DistanceUnit.INCH) + ", "
                    + storedLocation.getHeading(AngleUnit.DEGREES));
        } else {
            storedLocation = Field.redSmallZone;
            startingSide   = Field.Side.RED;
            goalPosition   = Field.redGoal;
            hw.lights.setLightColor(RGBLightController.RED);
            telemetry.addLine("Position not found. Defaulting to Red.");
        }

        hw.pinpoint.setPosition(storedLocation);
        hw.lights.setLightMode(RGBLightController.LEDMode.WAKE);

        totalRuntime.reset();
        readyRumbleTimer.reset();
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Init loop
    // -------------------------------------------------------------------------

    @Override
    public void init_loop() {
        hw.updateInitTeleOp();

        if (!testing) {
            telemetry.addLine(loaded
                    ? "Position found!"
                    : "Position not found. Defaulted to Red Far Zone.");
            telemetry.addLine("Position: "
                    + PoseUtils.poseToString(storedLocation, DistanceUnit.INCH, AngleUnit.DEGREES));
        } else {
            telemetry.addLine("Testing side: " + (testingSide == Field.Side.RED ? "Red" : "Blue"));
            telemetry.addLine("Press [Square] to switch sides.");
            if (gamepad1.xWasPressed()) {
                testingSide = (testingSide == Field.Side.RED) ? Field.Side.BLUE : Field.Side.RED;
                hw.lights.setLightColor(testingSide == Field.Side.BLUE
                        ? RGBLightController.BLUE : RGBLightController.RED);
            }
        }

        if (gamepad1.optionsWasPressed()) testing = !testing;

        telemetry.addLine("Test mode: " + testing + "  |  [options] to toggle.");
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Start
    // -------------------------------------------------------------------------

    @Override
    public void start() {
        if (testing) {
            storedLocation = (testingSide == Field.Side.RED)
                    ? Field.redSmallZone : Field.blueSmallZone;
            goalPosition   = (testingSide == Field.Side.RED)
                    ? Field.redGoal : Field.blueGoal;
            startingSide   = testingSide;
        }

        hw.pinpoint.setPosition(storedLocation);

        // Issue 3 fix: auto-aim is on from the very first loop — no button press needed
        hw.turret.isTargeting = true;

        // Flywheel spins up immediately so it's ready before the first scoring run
        hw.turret.spinUpFlywheel();

        totalRuntime.reset();
    }

    // -------------------------------------------------------------------------
    // Loop
    // -------------------------------------------------------------------------

    @Override
    public void loop() {

        // --- 1. Read pose first (Issue 1 fix: setTarget before updateTeleOp) ---
        pos = hw.pinpoint.getPosition();

        // Feed fresh pose into turret BEFORE updateTeleOp calls turret.update()
//        hw.turret.setTarget(pos, goalPosition);
        hw.updateTeleOp(this);
        hw.turret.updateFlywheelAndHood(pos, goalPosition);

        // --- 2. Cache all motor reads once — reused in telemetry (Issue 7 fix) ---
        distance         = hw.turret.getDistanceToTarget(hw.turret.offsetPoseToTurret(pos), goalPosition);
        leftVelocity     = hw.turret.launcherL.getVelocity();
        rightVelocity    = hw.turret.launcherR.getVelocity();
        avgVelocity      = (leftVelocity + rightVelocity) / 2.0;
        leftFlyCurrent   = hw.turret.launcherL.getCurrent(CurrentUnit.AMPS);
        rightFlyCurrent  = hw.turret.launcherR.getCurrent(CurrentUnit.AMPS);

        driveCurrentTotal = hw.drivetrain.leftFront.getCurrent(CurrentUnit.AMPS)
                + hw.drivetrain.leftBack.getCurrent(CurrentUnit.AMPS)
                + hw.drivetrain.rightFront.getCurrent(CurrentUnit.AMPS)
                + hw.drivetrain.rightBack.getCurrent(CurrentUnit.AMPS);
        intakeCurrent     = hw.intake.innerIntakeMotor.getCurrent(CurrentUnit.AMPS)
                + hw.intake.outerIntakeMotor.getCurrent(CurrentUnit.AMPS);
        turretCurrentTotal = leftFlyCurrent + rightFlyCurrent;
        totalRobotCurrent  = driveCurrentTotal + intakeCurrent + turretCurrentTotal;

        // --- 3. Pre-positioning (Issue 4 fix) ------------------------------------
        // Turret tracks and flywheel ramps up as soon as the robot is within
        // PRE_AIM_DISTANCE_INCHES of the goal — well before entering the zone.
        // When farther away, targeting stays on but flywheel idles to save current.
        if (distance <= PRE_AIM_DISTANCE_INCHES) {
            if (!hw.turret.isFlywheelSpinning) hw.turret.spinUpFlywheel();
        }

        // --- 4. Drivetrain ------------------------------------------------------
        hw.drivetrain.fieldOrientedDrive(this, pos, goalPosition,
                storedLocation.getHeading(AngleUnit.RADIANS), startingSide);

        // --- 5. Targeting toggle (triangle) — Issue 2 + 6 fix -------------------
        if (gamepad1.triangleWasPressed()) {
            hw.turret.isTargeting = !hw.turret.isTargeting;

            // Light mode tracks turret state using the correct variable
            if (totalRuntime.seconds() <= 110) {
                hw.lights.setLightMode(hw.turret.isTargeting
                        ? RGBLightController.LEDMode.FLASH
                        : RGBLightController.LEDMode.SOLID);
            }
        }

        // --- 6. Turret position --------------------------------------------------
        if (isParking) {
            hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));
            hw.intake.stop();
        } else if (hw.turret.isTargeting) {
            hw.turret.setTarget(pos, goalPosition);
        } else {
            hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));
        }

        // --- 7. Flywheel ready indicator (new) -----------------------------------
        // When flywheel is on target and turret is aiming, give driver a clear signal.
        boolean flywheelReady = hw.turret.isFlywheelSpinning
                && hw.turret.isTargeting
                && Math.abs(avgVelocity - hw.turret.velocity) < FLYWHEEL_READY_TOLERANCE;

        if (flywheelReady && !flywheelWasReady
                && readyRumbleTimer.milliseconds() > READY_RUMBLE_COOLDOWN_MS) {
            // Single short rumble + green LED = "ready to fire"
            gamepad1.rumbleBlips(1);
            gamepad1.setLedColor(0, 1, 0, 500);
            readyRumbleTimer.reset();
        }
        flywheelWasReady = flywheelReady;

        // --- 8. Firing (right trigger) ------------------------------------------
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.isForceIntaking = true;
            hw.intake.openGate();
        } else {
            hw.intake.isForceIntaking = false;
            hw.intake.closeGate();
        }

        // --- 9. Intake toggle (A) -----------------------------------------------
        if (gamepad1.aWasPressed()) hw.intake.toggle();

        // --- 10. Parking (right bumper) -----------------------------------------
        if (gamepad1.rightBumperWasPressed()) {
            switch (parkStatus) {
                case NOT_PARKED:
                    isParking = true;
                    hw.drivetrain.mobilePark();
                    parkStatus = ParkStatus.MOBILE_PARKED;
                    break;
                case MOBILE_PARKED:
                    hw.drivetrain.park();
                    parkStatus = ParkStatus.FULL_PARKED;
                    break;
                case FULL_PARKED:
                    isParking = false;
                    hw.drivetrain.unpark();
                    parkStatus = ParkStatus.NOT_PARKED;
                    break;
            }
        }

        // --- 11. Speed (left bumper hold) ----------------------------------------
        if (gamepad1.left_bumper) hw.drivetrain.slowDown();
        else                      hw.drivetrain.speedUp();

        // --- 12. Real-time goal adjustment (dpad) — Issue 5 fix -----------------
        // Fixed: now uses startingSide (was checking null 'side' variable)
        double goalX = goalPosition.getX(DistanceUnit.INCH);
        double goalY = goalPosition.getY(DistanceUnit.INCH);
        boolean goalChanged = false;

        if (startingSide == Field.Side.BLUE) {
            if      (gamepad1.dpadUpWasPressed())    { goalX -= 1; goalChanged = true; }
            else if (gamepad1.dpadDownWasPressed())  { goalX += 1; goalChanged = true; }
            else if (gamepad1.dpadLeftWasPressed())  { goalY -= 1; goalChanged = true; }
            else if (gamepad1.dpadRightWasPressed()) { goalY += 1; goalChanged = true; }
        } else {
            if      (gamepad1.dpadUpWasPressed())    { goalX += 1; goalChanged = true; }
            else if (gamepad1.dpadDownWasPressed())  { goalX -= 1; goalChanged = true; }
            else if (gamepad1.dpadLeftWasPressed())  { goalY += 1; goalChanged = true; }
            else if (gamepad1.dpadRightWasPressed()) { goalY -= 1; goalChanged = true; }
        }

        if (goalChanged) {
            goalPosition = new Pose2D(DistanceUnit.INCH, goalX, goalY,
                    AngleUnit.RADIANS, goalPosition.getHeading(AngleUnit.RADIANS));
        }

        // --- 13. Pinpoint position reset (options) ------------------------------
        if (gamepad1.optionsWasPressed()) {
            hw.pinpoint.setPosition(startingSide == Field.Side.BLUE
                    ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        // --- 14. Flywheel toggle (PS) -------------------------------------------
        if (gamepad1.psWasPressed()) hw.turret.toggleFlywheel();

        // --- 15. Endgame alerts -------------------------------------------------
        if (totalRuntime.seconds() > 100 && !endgame) {
            endgame = true;
            hw.lights.setLightColor(1);
            gamepad1.setLedColor(1, 1, 1, 1_000_000_000);
            gamepad1.rumbleBlips(4);
        }
        if (totalRuntime.seconds() > 110 && hw.lights.getLightColor() != 0.504) {
            hw.lights.setLightMode(RGBLightController.LEDMode.PULSE);
        }

        // --- 16. Telemetry — all values from cached reads (Issue 7 fix) ---------
        ptelemetry.setUpdateInterval(50);

        ptelemetry.addLine("--- flywheel ---");
        ptelemetry.addData("Target velocity",   String.format(Locale.US, "%.0f", hw.turret.velocity));
        ptelemetry.addData("Avg velocity",       String.format(Locale.US, "%.0f", avgVelocity));
        ptelemetry.addData("Left velocity",      String.format(Locale.US, "%.0f", leftVelocity));
        ptelemetry.addData("Right velocity",     String.format(Locale.US, "%.0f", rightVelocity));
        ptelemetry.addData("Left current (A)",   String.format(Locale.US, "%.2f", leftFlyCurrent));
        ptelemetry.addData("Right current (A)",  String.format(Locale.US, "%.2f", rightFlyCurrent));
        ptelemetry.addData("Ready to fire",      flywheelReady ? "YES" : "no");

        ptelemetry.addLine("--- hood + targeting ---");
        ptelemetry.addData("Hood target",        String.format(Locale.US, "%.3f", hw.turret.hoodTarget));
        ptelemetry.addData("Distance (in)",      String.format(Locale.US, "%.1f", distance));
        ptelemetry.addData("Targeting",          hw.turret.isTargeting ? "ON" : "off");
        ptelemetry.addData("Pre-aim active",     distance <= PRE_AIM_DISTANCE_INCHES ? "YES" : "no");

        ptelemetry.addLine("--- current draw ---");
        ptelemetry.addData("Drivetrain (A)",     String.format(Locale.US, "%.2f", driveCurrentTotal));
        ptelemetry.addData("Intake (A)",         String.format(Locale.US, "%.2f", intakeCurrent));
        ptelemetry.addData("Turret (A)",         String.format(Locale.US, "%.2f", turretCurrentTotal));
        ptelemetry.addData("TOTAL (A)",          String.format(Locale.US, "%.2f", totalRobotCurrent));

        ptelemetry.update();

        // Driver station telemetry — lean, only what the drive team needs mid-match
        telemetry.addLine("Targeting: " + (hw.turret.isTargeting ? "ON" : "off")   // Issue 2 fix
                + "  |  Ready: " + (flywheelReady ? "YES" : "no"));
        telemetry.addLine("Distance: " + String.format(Locale.US, "%.1f", distance) + " in"
                + "  |  Pre-aim: " + (distance <= PRE_AIM_DISTANCE_INCHES ? "YES" : "no"));
        telemetry.addLine("Continuous heading: " + String.format(Locale.US, "%.1f", hw.turret.continuousHeading));
        telemetry.addLine("Parked: " + hw.drivetrain.isParked);
        telemetry.addLine("Position: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addLine("Goal: " + String.format(Locale.US, "(%.1f, %.1f)",
                goalPosition.getX(DistanceUnit.INCH), goalPosition.getY(DistanceUnit.INCH)));
        telemetry.addLine("Time: " + String.format(Locale.US, "%.1f", totalRuntime.seconds()) + "s");
        telemetry.addLine("Total current: " + String.format(Locale.US, "%.2f", totalRobotCurrent) + " A");
        telemetry.update();
    }
}