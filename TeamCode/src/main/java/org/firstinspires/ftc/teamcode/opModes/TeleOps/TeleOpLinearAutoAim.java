package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PIDFControllerCTS;
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
 *  - While auto-aim is active the driver retains FULL translational control.
 *  - Drive is always field-centric mecanum, same as CTS.
 *  - Alliance side / goal coordinates loaded exactly the same way as CTS.
 *
 * PIDF TUNING VIA PANELS:
 *  - All heading PIDF gains and limits live in the inner @Configurable class
 *    HeadingPIDFConfig below.
 *  - Open the Configurables widget in Panels, find "HeadingPIDFConfig", and
 *    edit the values live — no reupload needed.
 *  - Coefficients are re-applied to the controller every loop, so changes take
 *    effect immediately.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "tele - Linear Auto Aim", group = "Robot")
public class TeleOpLinearAutoAim extends OpMode {

    // =========================================================================
    // Live-tunable PIDF config — edit these in the Panels Configurables widget
    // =========================================================================

    /**
     * All fields here are {@code public static} so Panels can read and write
     * them at runtime without any code reupload.
     *
     * IMPORTANT: The opmode reads these values fresh every loop and calls
     * headingPID.setPIDFCoefficients() each iteration, so any change you make
     * in the Panels dashboard takes effect on the very next control loop.
     * You will NOT need to restart the opmode after adjusting gains.
     *
     * Tuning guide (start here):
     *   1. Set Kp = 0.6, Ki = 0, Kd = 0, Kf = 0.
     *   2. Raise Kp in Panels until the robot oscillates when locking to goal,
     *      then back it off ~30%.
     *   3. Raise Kd to damp any remaining oscillation.
     *   4. Ki and Kf should stay at 0 for heading control in almost all cases.
     *   5. Adjust MAX_ROTATION_POWER if the correction feels too aggressive or
     *      too weak vs. the driver's translation inputs.
     *   6. Adjust HEADING_TOLERANCE_DEG if the robot micro-corrects constantly
     *      (raise it) or stops short of the goal heading (lower it).
     */

    @Configurable
    public static class HeadingPIDFConfig {

        /** Proportional gain — primary tuning knob. Start at 0.6. */
        @Sorter(sort = 1)
        public static double Kp = 0.60;

        /** Integral gain — leave at 0 for heading control. */
        @Sorter(sort = 2)
        public static double Ki = 0.00;

        /** Derivative gain — raise to damp oscillation after Kp is set. */
        @Sorter(sort = 3)
        public static double Kd = 0.08;

        /** Feed-forward gain — leave at 0; no gravity/friction to overcome. */
        @Sorter(sort = 4)
        public static double Kf = 0.00;

        /**
         * Maximum fraction of full motor power the rotation correction may use.
         * Range: [0.0, 1.0].  Lower this if auto-aim spins too hard vs. driving.
         */
        @Sorter(sort = 5)
        public static double MAX_ROTATION_POWER = 0.60;

        /**
         * Dead-band in DEGREES. The PID returns 0 when the heading error is
         * inside this window — prevents constant micro-corrections while aimed.
         * ~2–4° is a good range.
         */
        @Sorter(sort = 6)
        public static double HEADING_TOLERANCE_DEG = 2.9;
    }

    // =========================================================================
    // Hardware + subsystem state
    // =========================================================================

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private HardwareManager hw;
    private Pose2D goalPosition = null;

    private final ElapsedTime totalRuntime = new ElapsedTime();

    // Position / alliance
    private Pose2D     storedLocation = null;
    private Field.Side startingSide   = null;
    private boolean    loaded         = false;

    // Chassis auto-aim
    private boolean autoAimEnabled = false;

    /**
     * PIDF controller for chassis heading.
     * Gains are re-applied every loop from HeadingPIDFConfig so live edits
     * in Panels take effect immediately without restarting the opmode.
     */
    private PIDFControllerCTS headingPID;

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

        // Build the heading PIDF with the current (default) config values.
        headingPID = new PIDFControllerCTS(
                HeadingPIDFConfig.Kp,
                HeadingPIDFConfig.Ki,
                HeadingPIDFConfig.Kd,
                HeadingPIDFConfig.Kf,
                -HeadingPIDFConfig.MAX_ROTATION_POWER,
                HeadingPIDFConfig.MAX_ROTATION_POWER);
        headingPID.setTolerance(Math.toRadians(HeadingPIDFConfig.HEADING_TOLERANCE_DEG));

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
            if (gamepad1.xWasPressed()) {
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

        // --- 1. Re-apply PIDF coefficients from Panels every loop --------------
        //   This is the key pattern for live Panels tuning: always read the
        //   static config fields rather than caching them once at init.
        //   Any edit you make in the Panels Configurables widget is reflected
        //   on the very next iteration — no opmode restart needed.
        headingPID.setPIDFCoefficients(
                HeadingPIDFConfig.Kp,
                HeadingPIDFConfig.Ki,
                HeadingPIDFConfig.Kd,
                HeadingPIDFConfig.Kf);
        headingPID.setTolerance(Math.toRadians(HeadingPIDFConfig.HEADING_TOLERANCE_DEG));
        // Note: MAX_ROTATION_POWER is applied as a manual clamp below rather
        // than via the PIDFController constructor limits, so it also responds
        // to live Panels edits without rebuilding the controller object.

        // --- 2. Read pose -------------------------------------------------------
        pos = hw.pinpoint.getPosition();

        // --- 3. Keep turret locked at 0° every loop ----------------------------
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        // --- 4. Cache distance for telemetry -----------------------------------
        distance = hw.turret.getDistanceToTarget(
                hw.turret.offsetPoseToTurret(pos), goalPosition);

        // --- 5. [Square] toggles chassis auto-aim ------------------------------
        if (gamepad1.squareWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
            headingPID.reset(); // clear integral wind-up on every toggle

            hw.lights.setLightMode(autoAimEnabled
                    ? RGBLightController.LEDMode.FLASH
                    : RGBLightController.LEDMode.SOLID);
        }

        // --- 6. Compute heading correction -------------------------------------
        //   Target heading = angle from robot to goal in the field frame.
        //   Wrapped to [-π, π] so the robot always takes the shortest rotation.
        rotationCorrection = 0.0;
        headingErrorDeg    = 0.0;

        if (autoAimEnabled) {
            double robotX = pos.getX(DistanceUnit.INCH);
            double robotY = pos.getY(DistanceUnit.INCH);
            double goalX  = goalPosition.getX(DistanceUnit.INCH);
            double goalY  = goalPosition.getY(DistanceUnit.INCH);

            double targetHeadingRad  = Math.atan2(goalY - robotY, goalX - robotX);
            double currentHeadingRad = pos.getHeading(AngleUnit.RADIANS);
            double wrappedError      = wrapAngle(targetHeadingRad - currentHeadingRad);

            headingErrorDeg = Math.toDegrees(wrappedError);

            // Setpoint = 0, process variable = -error → positive output = CCW
            headingPID.setTarget(0.0);
            rotationCorrection = headingPID.calculate(-wrappedError);

            // Clamp to live MAX_ROTATION_POWER so Panels edits take effect here too
            rotationCorrection = Math.max(
                    -HeadingPIDFConfig.MAX_ROTATION_POWER,
                    Math.min(HeadingPIDFConfig.MAX_ROTATION_POWER, rotationCorrection));
        }

        // --- 7. Field-centric mecanum drive ------------------------------------
        if (autoAimEnabled) {
            driveFieldCentricWithAutoAim(rotationCorrection);
        } else {
            hw.drivetrain.fieldOrientedDrive(this, pos, goalPosition,
                    storedLocation.getHeading(AngleUnit.RADIANS), startingSide);
        }

        // --- 8. Update all other subsystems ------------------------------------
        hw.updateTeleOp(this);

        // --- 9. Intake toggle [A] ----------------------------------------------
        if (gamepad1.aWasPressed()) hw.intake.toggle();

        // --- 10. Firing [right trigger] ----------------------------------------
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.isForceIntaking = true;
            hw.intake.openGate();
        } else {
            hw.intake.isForceIntaking = false;
            hw.intake.closeGate();
        }

        // --- 11. Parking [right bumper] ----------------------------------------
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

        // --- 12. Speed [left bumper hold] --------------------------------------
        if (gamepad1.left_bumper) hw.drivetrain.slowDown();
        else                      hw.drivetrain.speedUp();

        // --- 13. Real-time goal adjustment [dpad] — same as CTS ---------------
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

        // --- 14. Pinpoint position reset [options] -----------------------------
        if (gamepad1.optionsWasPressed()) {
            hw.pinpoint.setPosition(startingSide == Field.Side.BLUE
                    ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        // --- 15. Endgame alerts ------------------------------------------------
        if (totalRuntime.seconds() > 100 && !endgame) {
            endgame = true;
            hw.lights.setLightColor(1);
            gamepad1.setLedColor(1, 1, 1, 1_000_000_000);
            gamepad1.rumbleBlips(4);
        }
        if (totalRuntime.seconds() > 110 && hw.lights.getLightColor() != 0.504) {
            hw.lights.setLightMode(RGBLightController.LEDMode.PULSE);
        }

        // --- 16. Telemetry -----------------------------------------------------
        ptelemetry.setUpdateInterval(50);

        ptelemetry.addLine("--- PEACOCK EMERGENCY MODE ---");
        ptelemetry.addData("Turret", "LOCKED (0°)");

        ptelemetry.addLine("--- chassis auto-aim ---");
        ptelemetry.addData("Auto-Aim",        autoAimEnabled ? "ON  [Square to toggle]" : "OFF [Square to toggle]");
        ptelemetry.addData("Heading error",   String.format(Locale.US, "%.1f°", headingErrorDeg));
        ptelemetry.addData("Rotation output", String.format(Locale.US, "%.3f", rotationCorrection));
        ptelemetry.addData("Distance (in)",   String.format(Locale.US, "%.1f", distance));

        ptelemetry.addLine("--- live PIDF gains (edit in Panels Configurables) ---");
        ptelemetry.addData("Kp",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Kp));
        ptelemetry.addData("Ki",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Ki));
        ptelemetry.addData("Kd",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Kd));
        ptelemetry.addData("Kf",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Kf));
        ptelemetry.addData("Max rotation pwr", String.format(Locale.US, "%.2f",  HeadingPIDFConfig.MAX_ROTATION_POWER));
        ptelemetry.addData("Tolerance (deg)",  String.format(Locale.US, "%.1f°", HeadingPIDFConfig.HEADING_TOLERANCE_DEG));

        ptelemetry.addLine("--- field positioning ---");
        ptelemetry.addData("xPosition", String.format(Locale.US, "%.2f", pos.getX(DistanceUnit.INCH)));
        ptelemetry.addData("yPosition", String.format(Locale.US, "%.2f", pos.getY(DistanceUnit.INCH)));
        ptelemetry.addData("Heading",   String.format(Locale.US, "%.2f°", pos.getHeading(AngleUnit.DEGREES)));
        ptelemetry.addLine("Position: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));

        ptelemetry.update();

        // Driver-station telemetry (lean)
        telemetry.addLine("** PEACOCK EMERGENCY — TURRET LOCKED **");
        telemetry.addLine("Auto-Aim: " + (autoAimEnabled ? "ON" : "OFF") + "  |  [Square] to toggle");
        telemetry.addLine("Heading error: " + String.format(Locale.US, "%.1f°", headingErrorDeg));
        telemetry.addLine("Kp=" + HeadingPIDFConfig.Kp + "  Kd=" + HeadingPIDFConfig.Kd);
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
     * Field-centric mecanum drive with chassis rotation override from the
     * auto-aim PIDF. Translation comes from the driver's left stick; rotation
     * is replaced by {@code aimRotation} so the robot faces the goal while
     * the driver steers freely anywhere on the field.
     */
    private void driveFieldCentricWithAutoAim(double aimRotation) {
        double y  = -gamepad1.left_stick_y;  // forward / backward
        double x  =  gamepad1.left_stick_x;  // strafe
        double rx =  aimRotation;            // rotation from PIDF

        double botHeading = pos.getHeading(AngleUnit.RADIANS);

        // Rotate translation vector into field frame
        double rotX =  x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY =  x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1; // slight strafe boost for mecanum imperfection

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        double scale = 1;
        hw.drivetrain.leftFront.setPower(frontLeftPower   * scale);
        hw.drivetrain.leftBack.setPower(backLeftPower     * scale);
        hw.drivetrain.rightFront.setPower(frontRightPower * scale);
        hw.drivetrain.rightBack.setPower(backRightPower   * scale);
    }

    /** Wraps an angle to [-π, π] — ensures the PID takes the shortest path. */
    private static double wrapAngle(double angleRad) {
        while (angleRad >  Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }
}