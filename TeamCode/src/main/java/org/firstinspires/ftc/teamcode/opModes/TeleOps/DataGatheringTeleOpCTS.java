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

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Gather Data - CTS", group="Robot")
public class DataGatheringTeleOpCTS extends OpMode {

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private HardwareManager hw = new HardwareManager(hardwareMap);
    private Pose2D goalPosition = null;

    ElapsedTime totalRuntime = new ElapsedTime();

    double storedHeadingDegrees = 90.0;
    Pose2D storedLocation;
    boolean loaded = false;
    Field.Side startingSide = null;
    boolean endgame = false;

    // Logging
    private FileWriter csvWriter = null;
    private int snapshotCount = 0;

    // --- Init ---------------------------------------------------------------

    @Override
    public void init() {
        hw.initTeleOp(hardwareMap);

        if (Field.lastKnownPosition != null) {
            if (Field.lastAllianceSide == Field.Side.BLUE) {
                hw.lights.setLightColor(RGBLightController.BLUE);
                goalPosition = Field.blueGoal;
                telemetry.addLine("Blue side.");
            } else if (Field.lastAllianceSide == Field.Side.RED) {
                hw.lights.setLightColor(RGBLightController.RED);
                goalPosition = Field.redGoal;
                telemetry.addLine("Red side.");
            } else {
                hw.lights.setLightColor(RGBLightController.RED);
                goalPosition = Field.redGoal;
                telemetry.addLine("Alliance side not found. Defaulting to Red.");
            }
            storedLocation = Field.lastKnownPosition;
            loaded = true;
            startingSide = Field.lastAllianceSide;
        } else {
            storedLocation = Field.redSmallZone;
            startingSide = Field.Side.RED;
            goalPosition = Field.redGoal;
            hw.lights.setLightColor(RGBLightController.RED);
            telemetry.addLine("Position not found. Defaulting to Red.");
        }

        storedHeadingDegrees = storedLocation.getHeading(AngleUnit.DEGREES);
        hw.pinpoint.setPosition(storedLocation);
        hw.lights.setLightMode(RGBLightController.LEDMode.WAKE);

        // Open CSV file on Control Hub storage
        try {
            String path = "/sdcard/FIRST/data_gather_" + System.currentTimeMillis() + ".csv";
            csvWriter = new FileWriter(path);
            csvWriter.write("snapshot,distance_in,target_velocity,avg_velocity,hood_target\n");
            telemetry.addLine("Logging to: " + path);
        } catch (IOException e) {
            telemetry.addLine("WARNING: Could not open CSV file — " + e.getMessage());
        }

        totalRuntime.reset();
        telemetry.update();
    }

    // --- Loop ---------------------------------------------------------------

    boolean testing = false;
    Field.Side testingSide = Field.Side.RED;

    @Override
    public void init_loop() {
        hw.updateTeleOp(this);
        if (loaded) {
            telemetry.addLine("Position found: " + PoseUtils.poseToString(storedLocation, DistanceUnit.INCH, AngleUnit.DEGREES));
        } else {
            telemetry.addLine("Position not found. Defaulted to Red Far Zone.");
        }

        if (gamepad1.optionsWasPressed()) testing = !testing;
        if (testing && gamepad1.xWasPressed()) {
            hw.lights.setLightColor((testingSide == Field.Side.RED) ? RGBLightController.BLUE : RGBLightController.RED);
            testingSide = (testingSide == Field.Side.RED) ? Field.Side.BLUE : Field.Side.RED;
        }

        telemetry.addLine("Test mode: " + testing + "  |  Press [options] to toggle.");
        telemetry.update();
    }

    @Override
    public void start() {
        if (testing) {
            storedLocation    = (testingSide == Field.Side.RED) ? Field.redSmallZone  : Field.blueSmallZone;
            goalPosition      = (testingSide == Field.Side.RED) ? Field.redGoal       : Field.blueGoal;
            startingSide      = (testingSide == Field.Side.RED) ? Field.Side.RED      : Field.Side.BLUE;
            storedHeadingDegrees = storedLocation.getHeading(AngleUnit.DEGREES);
        } else {
            goalPosition = Field.redGoal;
        }
        hw.pinpoint.setPosition(storedLocation);
        hw.turret.isTargeting = true;   // Bug 1 fix
    }

    private Pose2D pos;

    @Override
    public void loop() {
        pos = hw.pinpoint.getPosition();

        // Bug 2 fix: feed pose before updateTeleOp so turret.update() sees fresh data
        hw.turret.setTarget(pos, goalPosition);
        hw.updateTeleOp(this);
        hw.turret.updateFlywheelAndHood(pos, goalPosition);

        hw.drivetrain.fieldOrientedDrive(this, pos, goalPosition,
                storedLocation.getHeading(AngleUnit.RADIANS), startingSide);

        // --- Targeting toggle (Y) -------------------------------------------
        if (gamepad1.yWasPressed()) {
            hw.turret.isTargeting = !hw.turret.isTargeting;
            if (!(totalRuntime.seconds() > 110)) {
                hw.lights.setLightMode(hw.lights.getLightMode() == RGBLightController.LEDMode.SOLID
                        ? RGBLightController.LEDMode.FLASH
                        : RGBLightController.LEDMode.SOLID);
            }
        }

        // --- Hood adjust (dpad up/down) -------------------------------------
        if      (gamepad1.dpadUpWasPressed())   hw.turret.incrementHood(0.05);
        else if (gamepad1.dpadDownWasPressed())  hw.turret.incrementHood(-0.05);

        // --- Flywheel adjust (dpad left/right) ------------------------------
        if      (gamepad1.dpadLeftWasPressed())  hw.turret.incrementFlywheel(-50);
        else if (gamepad1.dpadRightWasPressed())  hw.turret.incrementFlywheel(50);

        // --- Flywheel toggle (PS button) ------------------------------------
        if (gamepad1.psWasPressed()) hw.turret.toggleFlywheel();

        // --- Firing (right trigger) -----------------------------------------
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.isForceIntaking = true;
            hw.intake.openGate();
        } else {
            hw.intake.isForceIntaking = false;
            hw.intake.closeGate();
        }

        if (gamepad1.aWasPressed()) hw.intake.toggle();

        // --- Speed (X hold) -------------------------------------------------
        if (gamepad1.x) hw.drivetrain.slowDown();
        else            hw.drivetrain.speedUp();

        // --- Reset position (options) ---------------------------------------
        if (gamepad1.optionsWasPressed()) {
            hw.pinpoint.pinpoint.setPosition(
                    startingSide == Field.Side.BLUE ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        // --- DATA SNAPSHOT (triangle / Y on PS controller) ------------------
        // Press triangle to log current distance, velocity, and hood to CSV.
        // Controller rumbles once to confirm. Count shown in telemetry.
        if (gamepad1.triangleWasPressed()) {
            logSnapshot();
        }

        // --- Endgame --------------------------------------------------------
        if (totalRuntime.seconds() > 100 && !endgame) {
            endgame = true;
            hw.lights.setLightColor(1);
            gamepad1.setLedColor(1, 1, 1, 1_000_000_000);
            gamepad1.rumbleBlips(4);
        }
        if (totalRuntime.seconds() > 110 && hw.lights.getLightColor() != 0.504) {
            hw.lights.setLightMode(RGBLightController.LEDMode.PULSE);
        }

        // --- Telemetry ------------------------------------------------------
        double distance = hw.turret.getDistanceToTarget(
                hw.turret.offsetPoseToTurret(pos), goalPosition);
        double avgVelocity = hw.turret.getAverageFlywheelVelocity();

        double leftVelocity  = hw.turret.launcherL.getVelocity();
        double rightVelocity = hw.turret.launcherR.getVelocity();
        double leftCurrent   = hw.turret.launcherL.getCurrent(CurrentUnit.AMPS);
        double rightCurrent  = hw.turret.launcherR.getCurrent(CurrentUnit.AMPS);

        double driveCurrentTotal = hw.drivetrain.leftFront.getCurrent(CurrentUnit.AMPS)
                + hw.drivetrain.leftBack.getCurrent(CurrentUnit.AMPS)
                + hw.drivetrain.rightFront.getCurrent(CurrentUnit.AMPS)
                + hw.drivetrain.rightBack.getCurrent(CurrentUnit.AMPS);
        double intakeCurrent     = hw.intake.innerIntakeMotor.getCurrent(CurrentUnit.AMPS)
                + hw.intake.outerIntakeMotor.getCurrent(CurrentUnit.AMPS);
        double turretCurrentTotal = leftCurrent + rightCurrent;
        double totalRobotCurrent  = driveCurrentTotal + intakeCurrent + turretCurrentTotal;

        ptelemetry.addLine("--- flywheel ---");
        ptelemetry.addData("Target velocity",        hw.turret.velocity);
        ptelemetry.addData("Avg velocity",           avgVelocity);
        ptelemetry.addData("Left velocity",          leftVelocity);
        ptelemetry.addData("Right velocity",         rightVelocity);
        ptelemetry.addData("Left current (A)",       String.format(Locale.US, "%.2f", leftCurrent));
        ptelemetry.addData("Right current (A)",      String.format(Locale.US, "%.2f", rightCurrent));

        ptelemetry.addLine("--- hood ---");
        ptelemetry.addData("Hood target",            String.format(Locale.US, "%.3f", hw.turret.hoodTarget));

        ptelemetry.addLine("--- targeting ---");
        ptelemetry.addData("Distance (in)",          String.format(Locale.US, "%.1f", distance));
        ptelemetry.addData("Snapshots logged",       snapshotCount);

        ptelemetry.addLine("--- current draw ---");
        ptelemetry.addData("Drivetrain (A)",         String.format(Locale.US, "%.2f", driveCurrentTotal));
        ptelemetry.addData("Intake (A)",             String.format(Locale.US, "%.2f", intakeCurrent));
        ptelemetry.addData("Turret (A)",             String.format(Locale.US, "%.2f", turretCurrentTotal));
        ptelemetry.addData("TOTAL (A)",              String.format(Locale.US, "%.2f", totalRobotCurrent));

        ptelemetry.update();

        telemetry.addData("Target velocity",  hw.turret.velocity);
        telemetry.addData("Avg velocity",     avgVelocity);
        telemetry.addLine("Hood:     " + String.format(Locale.US, "%.3f", hw.turret.hoodTarget));
        telemetry.addLine("Distance: " + String.format(Locale.US, "%.1f", distance) + " in");
        telemetry.addLine("Snapshots: " + snapshotCount + "  |  [triangle] to log");
        telemetry.addLine("Targeting: " + hw.turret.isTargeting + "  |  [Y] to toggle");
        telemetry.addLine("Position: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));

        telemetry.addLine("--- current draw ---");
        telemetry.addData("Drivetrain (A)",         String.format(Locale.US, "%.2f", driveCurrentTotal));
        telemetry.addData("Intake (A)",             String.format(Locale.US, "%.2f", intakeCurrent));
        telemetry.addData("Turret (A)",             String.format(Locale.US, "%.2f", turretCurrentTotal));
        telemetry.addData("TOTAL (A)",              String.format(Locale.US, "%.2f", totalRobotCurrent));
        telemetry.update();
    }

    // --- Stop ---------------------------------------------------------------

    @Override
    public void stop() {
        // Flush and close the CSV so nothing is lost
        if (csvWriter != null) {
            try {
                csvWriter.flush();
                csvWriter.close();
            } catch (IOException e) {
                telemetry.addLine("WARNING: Could not close CSV — " + e.getMessage());
                telemetry.update();
            }
        }
    }

    // --- Helpers ------------------------------------------------------------

    private void logSnapshot() {
        if (csvWriter == null) return;

        double distance    = hw.turret.getDistanceToTarget(hw.turret.offsetPoseToTurret(pos), goalPosition);
        double avgVelocity = hw.turret.getAverageFlywheelVelocity();

        try {
            csvWriter.write(String.format(Locale.US, "%d,%.2f,%.2f,%.2f,%.4f\n",
                    ++snapshotCount,
                    distance,
                    hw.turret.velocity,
                    avgVelocity,
                    hw.turret.hoodTarget));
            csvWriter.flush();  // write immediately in case of crash
        } catch (IOException e) {
            telemetry.addLine("Log error: " + e.getMessage());
        }

        // Single rumble confirms the snapshot was written
        gamepad1.rumbleBlips(1);
    }
}