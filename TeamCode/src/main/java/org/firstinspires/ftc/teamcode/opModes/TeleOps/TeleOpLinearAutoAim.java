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
import org.firstinspires.ftc.teamcode.Classes.PIDFController;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

import java.util.Locale;

/**
 * TeleOpLinearAutoAim — Emergency / backup teleop for Project Peacock.
 *
 * Auto-aim rotates the ENTIRE CHASSIS to face the goal using a heading PIDF.
 * [Square] toggles auto-aim ON / OFF. Full translational control is always
 * available regardless of auto-aim state.
 *
 * KEY ARCHITECTURE CHANGE (fixes oscillation):
 *   driveFieldCentricWithAutoAim() has been REMOVED. The teleop now routes
 *   all drive through hw.drivetrain.fieldOrientedDrive() exclusively, using
 *   the new hw.drivetrain.setExternalRotation() hook to inject the PIDF
 *   correction. This means one drive path, one rotation convention, no
 *   duplicate field-centric math that could fight itself.
 *
 * SIGN CHAIN (verified):
 *   wrappedError > 0  →  robot needs to turn CCW to face goal
 *   headingPID.setTarget(wrappedTarget), calculate(currentHeading)
 *   →  error = wrappedTarget - currentHeading = +wrappedError
 *   →  PID output > 0
 *   →  setExternalRotation(+value)
 *   →  robotCentricDrive rotate > 0
 *   →  left motors faster than right  →  CCW  ✓
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "tele - Linear Auto Aim", group = "Robot")
public class TeleOpLinearAutoAim extends OpMode {

    // =========================================================================
    // Live-tunable PIDF config — edit in Panels Configurables widget
    // =========================================================================

    /**
     * Tuning guide:
     *   1. Set Kp = 0.3, Ki = 0, Kd = 0, Kf = 0. MAX_ROTATION_POWER = 0.4.
     *   2. Enable auto-aim, place robot 45° off target, watch it settle.
     *   3. Raise Kp until you see overshoot/oscillation, then back off ~30%.
     *   4. Raise Kd slowly to damp any remaining oscillation.
     *   5. Ki and Kf should stay at 0 for heading control.
     *   6. If robot snaps too hard, lower MAX_ROTATION_POWER first before
     *      touching Kp — it's a safer first lever.
     */
    @Configurable
    public static class HeadingPIDFConfig {

        @Sorter(sort = 1)
        public static double Kp = 0.30;   // start LOW — raise gradually

        @Sorter(sort = 2)
        public static double Ki = 0.00;

        @Sorter(sort = 3)
        public static double Kd = 0.00;   // add only after Kp is settled

        @Sorter(sort = 4)
        public static double Kf = 0.00;

        @Sorter(sort = 5)
        public static double MAX_ROTATION_POWER = 0.40;  // first lever to tune

        @Sorter(sort = 6)
        public static double HEADING_TOLERANCE_DEG = 3.0;
    }

    // =========================================================================
    // Hardware + state
    // =========================================================================

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private HardwareManager hw;
    private Pose2D goalPosition = null;

    private final ElapsedTime totalRuntime = new ElapsedTime();

    private Pose2D     storedLocation = null;
    private Field.Side startingSide   = null;
    private boolean    loaded         = false;

    private boolean autoAimEnabled = false;

    private PIDFController headingPID;

    private boolean isParking = false;
    private enum ParkStatus { NOT_PARKED, MOBILE_PARKED, FULL_PARKED }
    private ParkStatus parkStatus = ParkStatus.NOT_PARKED;

    private boolean endgame = false;

    private boolean    testing     = false;
    private Field.Side testingSide = Field.Side.RED;

    // Cached per loop for telemetry
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
        } else {
            storedLocation = Field.redSmallZone;
            startingSide   = Field.Side.RED;
            goalPosition   = Field.redGoal;
            hw.lights.setLightColor(RGBLightController.RED);
            telemetry.addLine("Position not found. Defaulting to Red.");
        }

        hw.pinpoint.setPosition(storedLocation);
        hw.lights.setLightMode(RGBLightController.LEDMode.WAKE);

        headingPID = new PIDFController(
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
            telemetry.addLine(loaded ? "Position found!" : "Position not found. Defaulted to Red Far Zone.");
            telemetry.addLine("Position: " + PoseUtils.poseToString(storedLocation, DistanceUnit.INCH, AngleUnit.DEGREES));
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
            storedLocation = (testingSide == Field.Side.RED) ? Field.redSmallZone  : Field.blueSmallZone;
            goalPosition   = (testingSide == Field.Side.RED) ? Field.redGoal       : Field.blueGoal;
            startingSide   = testingSide;
        }

        hw.pinpoint.setPosition(storedLocation);
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        autoAimEnabled = false;
        hw.drivetrain.setExternalRotation(null);  // make sure override is clear
        headingPID.reset();

        totalRuntime.reset();
    }

    // =========================================================================
    // Loop
    // =========================================================================

    @Override
    public void loop() {

        // --- 1. Update PIDF gains from Panels every loop -----------------------
        headingPID.setPIDFCoefficients(
                HeadingPIDFConfig.Kp,
                HeadingPIDFConfig.Ki,
                HeadingPIDFConfig.Kd,
                HeadingPIDFConfig.Kf);
        headingPID.setTolerance(Math.toRadians(HeadingPIDFConfig.HEADING_TOLERANCE_DEG));

        // --- 2. Read pose -------------------------------------------------------
        pos = hw.pinpoint.getPosition();

        // --- 3. Keep turret locked at 0° ---------------------------------------
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        // --- 4. Distance for telemetry -----------------------------------------
        distance = hw.turret.getDistanceToTarget(
                hw.turret.offsetPoseToTurret(pos), goalPosition);

        // --- 5. [Square] toggles auto-aim --------------------------------------
        if (gamepad1.squareWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
            headingPID.reset();

            if (!autoAimEnabled) {
                hw.drivetrain.setExternalRotation(null);  // restore driver rotation
            }

            hw.lights.setLightMode(autoAimEnabled
                    ? RGBLightController.LEDMode.FLASH
                    : RGBLightController.LEDMode.SOLID);
        }

        // --- 6. Compute heading correction and inject into drivetrain ----------
        //
        // SIGN CHAIN:
        //   targetHeadingRad = atan2(goalY-robotY, goalX-robotX)
        //   wrappedError     = shortest angular distance robot must rotate
        //   wrappedTarget    = currentHeading + wrappedError
        //                      (keeps both values in the same numeric range
        //                       so PID error = wrappedTarget - currentHeading
        //                                    = wrappedError, correctly signed)
        //   PID output > 0   → rotate CCW → left motors faster → CCW ✓
        //
        rotationCorrection = 0.0;
        headingErrorDeg    = 0.0;

        if (autoAimEnabled) {
            double robotX = pos.getX(DistanceUnit.INCH);
            double robotY = pos.getY(DistanceUnit.INCH);
            double goalX  = goalPosition.getX(DistanceUnit.INCH);
            double goalY  = goalPosition.getY(DistanceUnit.INCH);

            double targetHeadingRad  = Math.atan2(goalY - robotY, goalX - robotX);
            double currentHeadingRad = pos.getHeading(AngleUnit.RADIANS);

            // Shortest-path error, always in [-π, π]
            double wrappedError = wrapAngle(targetHeadingRad - currentHeadingRad);
            headingErrorDeg = Math.toDegrees(wrappedError);

            // Shift target into the same numeric neighborhood as currentHeading
            // so the PID never sees a spurious ~2π jump in error
            double wrappedTarget = currentHeadingRad + wrappedError;

            headingPID.setTarget(wrappedTarget);
            rotationCorrection = headingPID.calculate(currentHeadingRad);

            // Apply live MAX_ROTATION_POWER clamp
            rotationCorrection = Math.max(
                    -HeadingPIDFConfig.MAX_ROTATION_POWER,
                    Math.min(HeadingPIDFConfig.MAX_ROTATION_POWER, rotationCorrection));

            // Inject into drivetrain — fieldOrientedDrive will use this instead
            // of the driver's right stick or the internal rotationPID
            hw.drivetrain.setExternalRotation(rotationCorrection);
        }

        // --- 7. Single unified drive path --------------------------------------
        //   Always calls the same fieldOrientedDrive. When auto-aim is ON the
        //   rotation axis is the PIDF correction (set above). When OFF it is
        //   the driver's right stick. No separate drive path, no duplicate math.
        hw.drivetrain.fieldOrientedDrive(this, pos, goalPosition,
                storedLocation.getHeading(AngleUnit.RADIANS), startingSide);

        // --- 8. Update subsystems ----------------------------------------------
        hw.updateTeleOp(this);

        // --- 9. Intake [A] -----------------------------------------------------
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

        // --- 13. Goal adjustment [dpad] ----------------------------------------
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

        // --- 14. Pinpoint reset [options] --------------------------------------
        if (gamepad1.optionsWasPressed()) {
            hw.pinpoint.setPosition(startingSide == Field.Side.BLUE
                    ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        // --- 15. Endgame -------------------------------------------------------
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
        ptelemetry.addData("Auto-Aim",        autoAimEnabled ? "ON  [Square]" : "OFF [Square]");
        ptelemetry.addData("Heading error",   String.format(Locale.US, "%.1f°", headingErrorDeg));
        ptelemetry.addData("Rotation output", String.format(Locale.US, "%.3f",  rotationCorrection));
        ptelemetry.addData("Distance (in)",   String.format(Locale.US, "%.1f",  distance));

        ptelemetry.addLine("--- PIDF (edit in Panels) ---");
        ptelemetry.addData("Kp",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Kp));
        ptelemetry.addData("Ki",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Ki));
        ptelemetry.addData("Kd",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Kd));
        ptelemetry.addData("Kf",               String.format(Locale.US, "%.4f", HeadingPIDFConfig.Kf));
        ptelemetry.addData("Max rotation pwr", String.format(Locale.US, "%.2f", HeadingPIDFConfig.MAX_ROTATION_POWER));
        ptelemetry.addData("Tolerance (deg)",  String.format(Locale.US, "%.1f", HeadingPIDFConfig.HEADING_TOLERANCE_DEG));

        ptelemetry.addLine("--- field position ---");
        ptelemetry.addData("X",       String.format(Locale.US, "%.2f", pos.getX(DistanceUnit.INCH)));
        ptelemetry.addData("Y",       String.format(Locale.US, "%.2f", pos.getY(DistanceUnit.INCH)));
        ptelemetry.addData("Heading", String.format(Locale.US, "%.2f°", pos.getHeading(AngleUnit.DEGREES)));

        ptelemetry.update();

        telemetry.addLine("** PEACOCK EMERGENCY — TURRET LOCKED **");
        telemetry.addLine("Auto-Aim: " + (autoAimEnabled ? "ON" : "OFF") + "  |  [Square]");
        telemetry.addLine("Error: "    + String.format(Locale.US, "%.1f°", headingErrorDeg)
                + "  Output: " + String.format(Locale.US, "%.3f",  rotationCorrection));
        telemetry.addLine("Kp=" + HeadingPIDFConfig.Kp + "  Kd=" + HeadingPIDFConfig.Kd);
        telemetry.addLine("Distance: " + String.format(Locale.US, "%.1f", distance) + " in");
        telemetry.addLine("Pos: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addLine("Goal: " + String.format(Locale.US, "(%.1f, %.1f)",
                goalPosition.getX(DistanceUnit.INCH), goalPosition.getY(DistanceUnit.INCH)));
        telemetry.addLine("Time: " + String.format(Locale.US, "%.1f", totalRuntime.seconds()) + "s");
        telemetry.update();
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /** Wraps an angle to [-π, π]. */
    private static double wrapAngle(double angleRad) {
        while (angleRad >  Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }
}