package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PIDFController;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

import java.util.Locale;

/**
 * TeleOpPeacock — Emergency / backup teleop for Project Peacock.
 *
 * KEY DIFFERENCES FROM TeleOpCTS:
 *  - Turret is LOCKED at 0° (forward) for the entire match — no turret movement.
 *  - Auto-aim rotates the ENTIRE CHASSIS to face the goal via a heading PIDF loop.
 *  - [Square] on Gamepad 1 toggles chassis auto-aim ON / OFF.
 *  - While auto-aim is active the driver retains FULL translational control (strafe/drive).
 *  - Drive is always field-centric mecanum, same as CTS.
 *  - Alliance side / goal coordinates loaded exactly the same way as CTS.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "tele - Linear Auto Aim", group = "Robot")
public class TeleOpLinearAutoAim extends OpMode {

    // =========================================================================
    // Tuning constants — adjust these on the field
    // =========================================================================

    /**
     * Heading PIDF gains for chassis auto-aim.
     *
     * Start with Kp=0.6, Ki=0.0, Kd=0.08, Kf=0.0 and tune from there.
     * The output is clamped to [-MAX_ROTATION_POWER, +MAX_ROTATION_POWER].
     *
     * Tuning guide:
     *  1. Raise Kp until the robot oscillates, then back off ~30%.
     *  2. Add Kd to damp oscillation.
     *  3. Ki should stay near 0 for heading control — heading error rarely
     *     accumulates in a meaningful way during teleop.
     */
    private static final double HEADING_Kp           =  0.60;
    private static final double HEADING_Ki           =  0.00;
    private static final double HEADING_Kd           =  0.08;
    private static final double HEADING_Kf           =  0.00;

    /** Maximum fraction of full power the rotation correction may use [0, 1]. */
    private static final double MAX_ROTATION_POWER   =  0.60;

    /**
     * Dead-band (radians) inside which the PID considers heading "on target"
     * and returns 0 — prevents constant micro-corrections.
     * ~2.9° is a good starting point.
     */
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(2.9);

    // =========================================================================
    // Hardware + subsystem state
    // =========================================================================

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private HardwareManager hw;
    private Pose2D goalPosition   = null;

    private final ElapsedTime totalRuntime = new ElapsedTime();

    // Position / alliance
    private Pose2D      storedLocation = null;
    private Field.Side  startingSide   = null;
    private boolean     loaded         = false;

    // Chassis auto-aim
    private boolean autoAimEnabled = false;

    /**
     * PIDF controller for chassis heading.
     * Output range [-MAX_ROTATION_POWER, +MAX_ROTATION_POWER].
     * Kf is 0 — a feed-forward term on a heading setpoint doesn't make physical
     * sense here; the robot isn't fighting gravity.
     */
    private PIDFController headingPID;

    // Parking state machine (mirrors CTS)
    private boolean isParking = false;
    private enum ParkStatus { NOT_PARKED, MOBILE_PARKED, FULL_PARKED }
    private ParkStatus parkStatus = ParkStatus.NOT_PARKED;

    // Endgame
    private boolean endgame = false;

    // Init-loop testing overrides (same pattern as CTS)
    private boolean    testing     = false;
    private Field.Side testingSide = Field.Side.RED;

    // Cached each loop
    private Pose2D pos;
    private double distance;
    private double headingErrorDeg;
    private double rotationCorrection;

    // =========================================================================
    // Init
    // =========================================================================

    @Override
    public void init() {
        hw = new HardwareManager(hardwareMap);
        hw.initTeleOp(hardwareMap);

        // --- Alliance + goal — identical logic to CTS ---
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

        // --- Heading PIDF ---
        headingPID = new PIDFController(
                HEADING_Kp, HEADING_Ki, HEADING_Kd, HEADING_Kf,
                -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
        headingPID.setTolerance(HEADING_TOLERANCE_RAD);

        totalRuntime.reset();
        telemetry.update();
    }

    // =========================================================================
    // Init loop
    // =========================================================================

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
            if (gamepad1.xWasPressed()) {   // xWasPressed = Square on PS layout
                testingSide = (testingSide == Field.Side.RED) ? Field.Side.BLUE : Field.Side.RED;
                hw.lights.setLightColor(testingSide == Field.Side.BLUE
                        ? RGBLightController.BLUE : RGBLightController.RED);
            }
        }

        if (gamepad1.optionsWasPressed()) testing = !testing;

        telemetry.addLine("Test mode: " + testing + "  |  [options] to toggle.");
        telemetry.addLine("Pinpoint Ready: " + hw.pinpoint.isReady());
        telemetry.addLine("PEACOCK EMERGENCY MODE — turret locked.");
        telemetry.update();
    }

    // =========================================================================
    // Start
    // =========================================================================

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

        // Lock turret at 0° (forward) — it will not move for the entire match.
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        // Auto-aim starts OFF — driver enables with [Square].
        autoAimEnabled = false;
        headingPID.reset();

        totalRuntime.reset();
    }

    // =========================================================================
    // Loop
    // =========================================================================

    @Override
    public void loop() {

        // --- 1. Read pose -------------------------------------------------------
        pos = hw.pinpoint.getPosition();

        // --- 2. Keep turret locked at 0° every loop ----------------------------
        //   We call this every loop so that even if something tries to move it,
        //   we immediately push it back to center.
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        // --- 3. Cache distance for telemetry -----------------------------------
        distance = hw.turret.getDistanceToTarget(
                hw.turret.offsetPoseToTurret(pos), goalPosition);

        // --- 4. [Square] toggles chassis auto-aim ------------------------------
        //   squareWasPressed maps to the PS Square button (same as xWasPressed
        //   in some SDK versions — use whichever your SDK exposes for Square).
        if (gamepad1.squareWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
            headingPID.reset(); // clear integral wind-up on every toggle

            hw.lights.setLightMode(autoAimEnabled
                    ? RGBLightController.LEDMode.FLASH
                    : RGBLightController.LEDMode.SOLID);
        }

        // --- 5. Compute heading correction -------------------------------------
        //   Target heading = angle from robot's current position to the goal,
        //   expressed in the same frame as Pinpoint's heading output (radians,
        //   field-centric, 0 = forward / positive-X axis).
        //
        //   We compute the desired chassis heading that points the robot's front
        //   at the goal, then run the PIDF on the error between that and the
        //   robot's actual heading.
        //
        //   The correction is only injected into the drivetrain when autoAimEnabled
        //   is true — otherwise it is 0 and the driver has full rotation control.

        rotationCorrection = 0.0;
        headingErrorDeg    = 0.0;

        if (autoAimEnabled) {
            double robotX  = pos.getX(DistanceUnit.INCH);
            double robotY  = pos.getY(DistanceUnit.INCH);
            double goalX   = goalPosition.getX(DistanceUnit.INCH);
            double goalY   = goalPosition.getY(DistanceUnit.INCH);

            // Angle from robot to goal in the field frame
            double targetHeadingRad = Math.atan2(goalY - robotY, goalX - robotX);

            // Current robot heading from Pinpoint
            double currentHeadingRad = pos.getHeading(AngleUnit.RADIANS);

            // Wrap error to [-π, π] to avoid spinning the long way around
            double rawError = targetHeadingRad - currentHeadingRad;
            double wrappedError = wrapAngle(rawError);

            headingErrorDeg = Math.toDegrees(wrappedError);

            // Feed the wrapped error as the "process variable" distance from 0.
            // We set the PID setpoint to 0 and feed it the negative error so that
            // a positive error (robot needs to turn CCW) gives a positive rotation
            // command (which is CCW in FTC field-centric drive convention).
            headingPID.setTarget(0.0);
            rotationCorrection = headingPID.calculate(-wrappedError);
        }

        // --- 6. Field-centric mecanum drive with optional rotation override -----
        //   fieldOrientedDrive already handles joystick → motor power mapping.
        //   When auto-aim is on we hijack the rotation axis by injecting
        //   rotationCorrection.  We do this BEFORE calling fieldOrientedDrive
        //   by overriding gamepad1.right_stick_x at the drivetrain level.
        //
        //   Because we cannot directly override the gamepad axis, we drive the
        //   mecanum wheels ourselves when auto-aim is active so translation and
        //   rotation blend correctly.  When auto-aim is off we delegate entirely
        //   to the existing fieldOrientedDrive call so nothing else changes.

        if (autoAimEnabled) {
            driveFieldCentricWithAutoAim(rotationCorrection);
        } else {
            hw.drivetrain.fieldOrientedDrive(this, pos, goalPosition,
                    storedLocation.getHeading(AngleUnit.RADIANS), startingSide);
        }

        // --- 7. Update all other subsystems ------------------------------------
        hw.updateTeleOp(this);

        // --- 8. Intake toggle [A] ----------------------------------------------
        if (gamepad1.aWasPressed()) hw.intake.toggle();

        // --- 9. Firing [right trigger] -----------------------------------------
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.isForceIntaking = true;
            hw.intake.openGate();
        } else {
            hw.intake.isForceIntaking = false;
            hw.intake.closeGate();
        }

        // --- 10. Parking [right bumper] ----------------------------------------
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

        // --- 11. Speed [left bumper hold] --------------------------------------
        if (gamepad1.left_bumper) hw.drivetrain.slowDown();
        else                      hw.drivetrain.speedUp();

        // --- 12. Real-time goal adjustment [dpad] — same logic as CTS ----------
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

        // --- 13. Pinpoint position reset [options] -----------------------------
        if (gamepad1.optionsWasPressed()) {
            hw.pinpoint.setPosition(startingSide == Field.Side.BLUE
                    ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        // --- 14. Endgame alerts ------------------------------------------------
        if (totalRuntime.seconds() > 100 && !endgame) {
            endgame = true;
            hw.lights.setLightColor(1);
            gamepad1.setLedColor(1, 1, 1, 1_000_000_000);
            gamepad1.rumbleBlips(4);
        }
        if (totalRuntime.seconds() > 110 && hw.lights.getLightColor() != 0.504) {
            hw.lights.setLightMode(RGBLightController.LEDMode.PULSE);
        }

        // --- 15. Telemetry -----------------------------------------------------
        ptelemetry.setUpdateInterval(50);

        ptelemetry.addLine("--- PEACOCK EMERGENCY MODE ---");
        ptelemetry.addData("Turret",         "LOCKED (0°)");

        ptelemetry.addLine("--- chassis auto-aim ---");
        ptelemetry.addData("Auto-Aim",        autoAimEnabled ? "ON  [Square to toggle]" : "OFF [Square to toggle]");
        ptelemetry.addData("Heading error",   String.format(Locale.US, "%.1f°", headingErrorDeg));
        ptelemetry.addData("Rotation output", String.format(Locale.US, "%.3f", rotationCorrection));
        ptelemetry.addData("Distance (in)",   String.format(Locale.US, "%.1f", distance));

        ptelemetry.addLine("--- field positioning ---");
        ptelemetry.addData("xPosition",       String.format(Locale.US, "%.2f", pos.getX(DistanceUnit.INCH)));
        ptelemetry.addData("yPosition",       String.format(Locale.US, "%.2f", pos.getY(DistanceUnit.INCH)));
        ptelemetry.addData("Heading",         String.format(Locale.US, "%.2f°", pos.getHeading(AngleUnit.DEGREES)));
        ptelemetry.addLine("Position: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));

        ptelemetry.update();

        // Driver-station telemetry (lean)
        telemetry.addLine("** PEACOCK EMERGENCY — TURRET LOCKED **");
        telemetry.addLine("Auto-Aim: " + (autoAimEnabled ? "ON" : "OFF") + "  |  [Square] to toggle");
        telemetry.addLine("Heading error: " + String.format(Locale.US, "%.1f°", headingErrorDeg));
        telemetry.addLine("Distance: " + String.format(Locale.US, "%.1f", distance) + " in");
        telemetry.addLine("Position: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addLine("Goal: " + String.format(Locale.US, "(%.1f, %.1f)",
                goalPosition.getX(DistanceUnit.INCH), goalPosition.getY(DistanceUnit.INCH)));
        telemetry.addLine("Parked: " + hw.drivetrain.isParked);
        telemetry.addLine("Time: " + String.format(Locale.US, "%.1f", totalRuntime.seconds()) + "s");
        telemetry.update();
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    /**
     * Field-centric mecanum drive with a chassis rotation override injected
     * from the auto-aim PIDF.
     *
     * Translation comes entirely from the driver's left stick (same as normal
     * field-centric drive). Rotation is replaced by {@code aimRotation} so the
     * robot always faces the goal while still going wherever the driver steers.
     *
     * The right stick X is intentionally ignored while auto-aim is active —
     * the driver cannot fight the heading lock. If you want the driver to be
     * able to partially override, you can blend:
     *   double rx = aimRotation + gamepad1.right_stick_x * DRIVER_ROTATION_BLEND;
     * and re-clamp to [-1, 1].
     *
     * @param aimRotation  Rotation power from the heading PIDF [-1, 1],
     *                     positive = counter-clockwise in FTC convention.
     */
    private void driveFieldCentricWithAutoAim(double aimRotation) {
        // Raw stick inputs
        double y  = -gamepad1.left_stick_y;   // forward / backward  (negate: up = positive)
        double x  =  gamepad1.left_stick_x;   // strafe left / right
        double rx =  aimRotation;             // rotation — from PIDF, not driver

        // Robot heading from Pinpoint for field-centric rotation
        double botHeading = pos.getHeading(AngleUnit.RADIANS);

        // Rotate the translation vector into the field frame
        double rotX =  x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY =  x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Slight boost to strafe to compensate for mecanum imperfection
        rotX *= 1.1;

        // Denominator ensures no motor exceeds 1.0
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // Respect slow-mode (left bumper hold sets a power scalar in hw.drivetrain)
//        double scale = hw.drivetrain.getCurrentSpeedScale(); // expose this getter if not present
        double scale = 1;       // setting this as a fixed value
        hw.drivetrain.leftFront.setPower(frontLeftPower  * scale);
        hw.drivetrain.leftBack.setPower(backLeftPower    * scale);
        hw.drivetrain.rightFront.setPower(frontRightPower * scale);
        hw.drivetrain.rightBack.setPower(backRightPower  * scale);
    }

    /**
     * Wraps an angle to the range [-π, π].
     * Used to ensure the PID always takes the shortest path to the target heading.
     */
    private static double wrapAngle(double angleRad) {
        while (angleRad >  Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }
}