package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
 * TeleOpLinearAutoAim — Peacock emergency teleop.
 *
 * DRIVE:
 *   Standard goBILDA mecanum field-centric drive.
 *   Left stick = translate, Right stick = rotate.
 *   Left trigger held = slow mode.
 *
 * AUTO-AIM:
 *   [Square] toggles chassis auto-aim. When ON the right stick rotation is
 *   replaced by a P controller that points the BACK of the robot at the goal.
 *   The driver retains full translational control at all times.
 *
 * TURRET:
 *   Locked at 0° (forward) for the entire match.
 *   [Triangle] toggles turret targeting (hw.turret.isTargeting).
 *   [PS] toggles flywheel.
 *
 * TUNING (in Panels Configurables → HeadingPConfig):
 *   Start with Kp = 1.0, everything else 0.
 *   If robot oscillates: lower Kp.
 *   If robot is sluggish: raise Kp.
 *   Add Kd only after Kp is stable — start at 0.05 and raise slowly.
 *   TOLERANCE_DEG controls the dead-band — 3° is a good starting point.
 */
@TeleOp(name = "tele - Linear Auto Aim", group = "Robot")
public class TeleOpLinearAutoAim extends OpMode {

    // =========================================================================
    // Tunable constants — edit live in Panels Configurables → HeadingPConfig
    // =========================================================================

    @Configurable
    public static class HeadingPConfig {

        /** P gain. Start at 1.0. Lower if oscillating, raise if sluggish. */
        @Sorter(sort = 1)
        public static double Kp = 1.0;

        /** D gain. Start at 0. Raise slowly to damp overshoot. */
        @Sorter(sort = 2)
        public static double Kd = 0.0;

        /** I gain. Leave at 0 for heading control. */
        @Sorter(sort = 3)
        public static double Ki = 0.0;

        /**
         * Hard cap on the rotation power the auto-aim may use [0, 1].
         * Lower this first if the robot is too aggressive.
         * 0.5 is a safe starting point.
         */
        @Sorter(sort = 4)
        public static double MAX_TURN_POWER = 0.5;

        /**
         * Dead-band in degrees. Robot stops correcting when error is smaller
         * than this. 3° is a good starting point.
         */
        @Sorter(sort = 5)
        public static double TOLERANCE_DEG = 3.0;
    }

    // =========================================================================
    // Hardware
    // =========================================================================

    private HardwareManager hw;

    // Drive motors — accessed directly for the auto-aim drive path
    private DcMotorEx motorFL, motorFR, motorBL, motorBR;

    // =========================================================================
    // State
    // =========================================================================

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Pose2D     goalPosition   = null;
    private Pose2D     storedLocation = null;
    private Field.Side startingSide   = null;
    private boolean    loaded         = false;

    private boolean autoAimEnabled = false;

    // Heading controller — P only to start, D available in Panels
    private PIDFController headingPID;

    private boolean    testing     = false;
    private Field.Side testingSide = Field.Side.RED;

    private boolean endgame = false;

    private boolean isParking = false;
    private enum ParkStatus { NOT_PARKED, MOBILE_PARKED, FULL_PARKED }
    private ParkStatus parkStatus = ParkStatus.NOT_PARKED;

    private final ElapsedTime runtime = new ElapsedTime();

    // Cached per loop for telemetry
    private Pose2D pos;
    private double headingErrorDeg  = 0;
    private double rotationOutput   = 0;
    private double distance         = 0;

    // =========================================================================
    // Init
    // =========================================================================

    @Override
    public void init() {
        hw = new HardwareManager(hardwareMap);
        hw.initTeleOp(hardwareMap);

        // Grab motor references directly so we can set powers in one place
        motorFL = hw.drivetrain.leftFront;
        motorFR = hw.drivetrain.rightFront;
        motorBL = hw.drivetrain.leftBack;
        motorBR = hw.drivetrain.rightBack;

        // Alliance + goal
        if (Field.lastKnownPosition != null) {
            startingSide = Field.lastAllianceSide;
            if (startingSide == Field.Side.BLUE) {
                goalPosition = Field.blueGoalLocal;
                hw.lights.setLightColor(RGBLightController.BLUE);
                telemetry.addLine("Blue side.");
            } else if (startingSide == Field.Side.RED) {
                goalPosition = Field.redGoalLocal;
                hw.lights.setLightColor(RGBLightController.RED);
                telemetry.addLine("Red side.");
            } else {
                startingSide = Field.Side.RED;
                goalPosition = Field.redGoalLocal;
                hw.lights.setLightColor(RGBLightController.RED);
                telemetry.addLine("Alliance unknown — defaulting Red.");
            }
            storedLocation = Field.lastKnownPosition;
            loaded = true;
        } else {
            startingSide   = Field.Side.RED;
            goalPosition   = Field.redGoalLocal;
            storedLocation = Field.redSmallZone;
            hw.lights.setLightColor(RGBLightController.RED);
            telemetry.addLine("No saved position — defaulting Red.");
        }

        hw.pinpoint.setPosition(storedLocation);
        hw.lights.setLightMode(RGBLightController.LEDMode.WAKE);

        // Build PID — constructor limits are just a safety net;
        // MAX_TURN_POWER is applied as an explicit clamp every loop.
        headingPID = new PIDFController(
                HeadingPConfig.Kp, HeadingPConfig.Ki, HeadingPConfig.Kd, 0.0,
                -1.0, 1.0);
        headingPID.setTolerance(Math.toRadians(HeadingPConfig.TOLERANCE_DEG));

        runtime.reset();
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
        telemetry.addLine("[Options] test mode: " + testing);
        telemetry.addLine("Pinpoint ready: " + hw.pinpoint.isReady());
        telemetry.addLine("PEACOCK — turret locked.");
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
                    ? Field.redGoalLocal : Field.blueGoalLocal;
            startingSide   = testingSide;
        }

        hw.pinpoint.setPosition(storedLocation);
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        autoAimEnabled = false;
        headingPID.reset();
        runtime.reset();
    }

    // =========================================================================
    // Loop
    // =========================================================================

    @Override
    public void loop() {

        // 1. Update PIDF gains from Panels every loop so changes are instant
        headingPID.setPIDFCoefficients(
                HeadingPConfig.Kp,
                HeadingPConfig.Ki,
                HeadingPConfig.Kd,
                0.0);
        headingPID.setTolerance(Math.toRadians(HeadingPConfig.TOLERANCE_DEG));

        // 2. Read pose
        pos = hw.pinpoint.getPosition();

        // 3. Cache distance for firing logic
        distance = hw.turret.getDistanceToTarget(hw.turret.offsetPoseToTurret(pos), goalPosition);

        // 4. Lock turret
        hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));

        // 5. Toggle chassis auto-aim with Square
        if (gamepad1.squareWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
            headingPID.reset();
            hw.lights.setLightMode(autoAimEnabled
                    ? RGBLightController.LEDMode.FLASH
                    : RGBLightController.LEDMode.SOLID);
        }

        // 6. Toggle turret targeting with Triangle
        if (gamepad1.triangleWasPressed()) {
            hw.turret.isTargeting = !hw.turret.isTargeting;
        }

        // 7. Toggle flywheel with PS
        if (gamepad1.psWasPressed()) hw.turret.toggleFlywheel();

        // 8. Compute rotation — either from auto-aim PID or driver stick
        double currentHeading = pos.getHeading(AngleUnit.RADIANS);

        if (autoAimEnabled) {
            // Angle FROM robot TO goal
            double angleToGoal = Math.atan2(
                    goalPosition.getX(DistanceUnit.INCH) - pos.getX(DistanceUnit.INCH),
                    goalPosition.getY(DistanceUnit.INCH) - pos.getY(DistanceUnit.INCH));

            // We want the BACK of the robot facing the goal.
            // Red and blue use opposite coordinate orientations so the target
            // heading offset is alliance-specific.
            double targetHeading = angleToGoal + Math.PI;

            // Wrap target into the same circle as currentHeading so the PID
            // always sees the shortest path and never a ±360° jump
            double wrappedTarget = currentHeading + wrapAngle(targetHeading - currentHeading);

            headingErrorDeg = Math.toDegrees(wrapAngle(targetHeading - currentHeading));

            headingPID.setTarget(wrappedTarget);
            rotationOutput = headingPID.calculate(currentHeading);

            // Hard cap
            rotationOutput = Math.max(-HeadingPConfig.MAX_TURN_POWER,
                    Math.min( HeadingPConfig.MAX_TURN_POWER, rotationOutput));
        } else {
            headingErrorDeg = 0;
            rotationOutput  = 0;
        }

        // 9. Field-centric mecanum drive
        //    When auto-aim is ON:  rotation = PIDF output, right stick ignored
        //    When auto-aim is OFF: rotation = right stick
        double drive  = (startingSide == Field.Side.RED)
                ? -gamepad1.left_stick_y
                : -gamepad1.left_stick_y;
        double strafe = (startingSide == Field.Side.RED)
                ? -gamepad1.left_stick_x * 1.1
                : -gamepad1.left_stick_x * 1.1;

        double rotate = autoAimEnabled
                ? rotationOutput
                : gamepad1.right_stick_x;


        // Alliance offset so field-centric "forward" is always away from
        // the driver, regardless of which side of the field the robot starts on
        double allianceOffset = (startingSide == Field.Side.RED)
                ? 3.0 * Math.PI / 2.0 : -Math.PI / 2.0;

        // Rotate the translation vector into the field frame
        double sinH = Math.sin(-(currentHeading + allianceOffset));
        double cosH = Math.cos(-(currentHeading + allianceOffset));

        double fieldX =  drive ;
        double fieldY =  strafe ;

        double rotX = fieldX * cosH - fieldY * sinH;
        double rotY = fieldX * sinH + fieldY * cosH;

        // Mix into wheel powers
        double fl = rotY + rotX + rotate;
        double fr = rotY - rotX - rotate;
        double bl = rotY - rotX + rotate;
        double br = rotY + rotX - rotate;

        // Normalize so no wheel exceeds 1.0
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        // Left trigger = slow mode (matches CTS)
        double speed = (gamepad1.left_trigger > 0.1)
                ? hw.drivetrain.SLOW_DRIVING_SPEED
                : 1.0;

        motorFL.setPower(fl / max * speed);
        motorFR.setPower(fr / max * speed);
        motorBL.setPower(bl / max * speed);
        motorBR.setPower(br / max * speed);

        // 10. Update subsystems
        hw.updateTeleOp(this);

        hw.turret.updateFlywheelAndHood(pos, goalPosition);

        // 11. Intake [A]
        if (gamepad1.aWasPressed()) hw.intake.toggle();

        // 12. Firing [right trigger] — with distance-based partial intake (matches CTS)
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.isForceIntaking = true;
            hw.intake.openGate();
            if (distance > 100) {
                hw.intake.partialIntakeTeleop();
            } else {
                hw.intake.openGate();
            }
        } else {
            hw.intake.isForceIntaking = false;
            hw.intake.closeGate();
        }

        // 13. Parking [right bumper]
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

        // 14. Goal fine-adjustment [dpad]
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

        // 15. Pinpoint reset [options]
        if (gamepad1.optionsWasPressed()) {
            hw.pinpoint.setPosition(startingSide == Field.Side.BLUE
                    ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        // 16. Endgame
        if (runtime.seconds() > 100 && !endgame) {
            endgame = true;
            hw.lights.setLightColor(1);
            gamepad1.setLedColor(1, 1, 1, 1_000_000_000);
            gamepad1.rumbleBlips(4);
        }
        if (runtime.seconds() > 110 && hw.lights.getLightColor() != 0.504) {
            hw.lights.setLightMode(RGBLightController.LEDMode.PULSE);
        }

        // 17. Telemetry
        ptelemetry.setUpdateInterval(50);
        ptelemetry.addLine("--- PEACOCK — TURRET LOCKED ---");
        ptelemetry.addData("Auto-Aim",            autoAimEnabled ? "ON [Square]" : "OFF [Square]");
        ptelemetry.addData("Turret targeting",    hw.turret.isTargeting ? "ON [Triangle]" : "OFF [Triangle]");
        ptelemetry.addData("Heading error (deg)", String.format(Locale.US, "%.1f", headingErrorDeg));
        ptelemetry.addData("Rotation output",     String.format(Locale.US, "%.3f", rotationOutput));
        ptelemetry.addLine("--- PIDF (Panels: HeadingPConfig) ---");
        ptelemetry.addData("Kp",             HeadingPConfig.Kp);
        ptelemetry.addData("Kd",             HeadingPConfig.Kd);
        ptelemetry.addData("MAX_TURN_POWER", HeadingPConfig.MAX_TURN_POWER);
        ptelemetry.addData("TOLERANCE_DEG",  HeadingPConfig.TOLERANCE_DEG);
        ptelemetry.addLine("--- Position ---");
        ptelemetry.addData("X",       String.format(Locale.US, "%.2f", pos.getX(DistanceUnit.INCH)));
        ptelemetry.addData("Y",       String.format(Locale.US, "%.2f", pos.getY(DistanceUnit.INCH)));
        ptelemetry.addData("Heading", String.format(Locale.US, "%.1f°", Math.toDegrees(currentHeading)));
        ptelemetry.addData("Distance (in)", String.format(Locale.US, "%.1f", distance));
        ptelemetry.addData("Goal", String.format(Locale.US, "(%.1f, %.1f)",
                goalPosition.getX(DistanceUnit.INCH), goalPosition.getY(DistanceUnit.INCH)));
        ptelemetry.update();

        telemetry.addLine("PEACOCK — TURRET LOCKED");
        telemetry.addLine("Auto-Aim: " + (autoAimEnabled ? "ON" : "OFF") + "  [Square]");
        telemetry.addLine("Turret: " + (hw.turret.isTargeting ? "ON" : "OFF") + "  [Triangle]");
        telemetry.addLine("Error: " + String.format(Locale.US, "%.1f°", headingErrorDeg)
                + "  Out: " + String.format(Locale.US, "%.3f", rotationOutput));
        telemetry.addLine("Kp=" + HeadingPConfig.Kp + "  Kd=" + HeadingPConfig.Kd
                + "  Max=" + HeadingPConfig.MAX_TURN_POWER);
        telemetry.addLine("Pos: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addLine("Goal: " + String.format(Locale.US, "(%.1f, %.1f)",
                goalPosition.getX(DistanceUnit.INCH), goalPosition.getY(DistanceUnit.INCH)));
        telemetry.update();
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /**
     * Wraps an angle into [-π, π].
     */
    private static double wrapAngle(double radians) {
        while (radians >  Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}