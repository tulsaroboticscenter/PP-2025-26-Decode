package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Robot")
public class TeleOp extends OpMode
{
    private Field.Side side = null;
    private HardwareManager hw = new HardwareManager();
    private Pose2D goalPosition = null;

    // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
    ElapsedTime totalRuntime = new ElapsedTime();
    ElapsedTime targetingDelayRuntime = new ElapsedTime();
    ElapsedTime targetingRefreshRuntime = new ElapsedTime();
    ElapsedTime velocityAdjustmentRuntime = new ElapsedTime();
    ElapsedTime robotLeadRuntime = new ElapsedTime();
    ElapsedTime intakeToggleRuntime = new ElapsedTime();
    ElapsedTime hoodToggleRuntime = new ElapsedTime();
    ElapsedTime flywheelToggleRuntime = new ElapsedTime();
    ElapsedTime endgameTickRuntime = new ElapsedTime();



    private boolean isTargeting = false;
    boolean isIntaking = false;
    double storedHeadingDegrees = 90.0;
    double botHeading = 0.0;
    Pose2D storedLocation;
    boolean loaded = false;
    Pose2D startingPosition = null;
    double fieldCentricOffset = 0;

    @Override
    public void init()
    {
        hw.initTeleOp(hardwareMap);
        if (Field.lastKnownPosition != null)
        {
            storedLocation = Field.lastKnownPosition;
            loaded = true;
            telemetry.addLine("Last known position found: " + storedLocation.getX(DistanceUnit.INCH) + ", " + storedLocation.getY(DistanceUnit.INCH) + ", " + storedLocation.getHeading(AngleUnit.DEGREES));
        }
        else
        {
            storedLocation = Field.redSmallZone;
            telemetry.addLine("Last known position not found. Defaulting to Red.");
        }
        if (Field.lastAllianceSide == Field.Side.BLUE)
        {
            hw.lights.setLightColor(0.611);
            goalPosition = Field.blueGoal;
            telemetry.addLine("Blue Side.");
        }
        else if (Field.lastAllianceSide == Field.Side.RED)
        {
            hw.lights.setLightColor(0.28);
            goalPosition = Field.redGoal;
            telemetry.addLine("Red Side.");
        }
        else
        {
            hw.lights.setLightColor(0.28);
            goalPosition = Field.redGoal;
            telemetry.addLine("Last alliance side not found. Defaulting to Red.");
        }
        storedHeadingDegrees = storedLocation.getHeading(AngleUnit.DEGREES);
        hw.pinpoint.setPosition(storedLocation);

        hw.lights.setLightMode(RGBLightController.LEDMode.WAKE);

        totalRuntime.reset();
        targetingDelayRuntime.reset();
        targetingRefreshRuntime.reset();
        velocityAdjustmentRuntime.reset();
        robotLeadRuntime.reset();
        intakeToggleRuntime.reset();
        hoodToggleRuntime.reset();
        flywheelToggleRuntime.reset();
        telemetry.update();
    }

    boolean testing = false;
    Field.Side testingSide = Field.Side.RED;

    @Override
    public void init_loop()
    {
        hw.updateTeleOp();
        if (!testing)
        {
            if (loaded)
            {
                telemetry.addLine("Position found!");
            }
            else
            {
                telemetry.addLine("Position not found. Defaulted to Red Far Zone");
            }
            telemetry.addLine("Position: " + PoseUtils.poseToString(storedLocation, DistanceUnit.INCH, AngleUnit.DEGREES));
        }
        else
        {

            telemetry.addLine("Testing side: " + ((testingSide == Field.Side.RED) ? "Red" : "Blue"));
            telemetry.addLine("Press [Square] to switch sides.");
            telemetry.addLine("Robot will start at the far zone of the selected side at startup.");
            if (gamepad1.xWasPressed())
            {
                // i love ternary operators
                hw.lights.setLightColor((testingSide == Field.Side.RED) ? 0.611 : 0.28);
                testingSide = ((testingSide == Field.Side.RED) ? Field.Side.BLUE : Field.Side.RED);
            }
        }
        if (gamepad1.optionsWasPressed())
        {
            testing = !testing;
        }

        telemetry.addLine("Test Driving: " + testing);
        telemetry.addLine("Press [option] to toggle test drive mode.");
    }

    @Override
    public void start()
    {
        hw.turret.initHood();
        if (testing)
        {
            if (testingSide == Field.Side.RED)
            {
                storedLocation = Field.redSmallZone;
                goalPosition = Field.redGoal;
                fieldCentricOffset = 90;
            }
            else
            {
                storedLocation = Field.blueSmallZone;
                goalPosition = Field.blueGoal;
                fieldCentricOffset = -90;
            }
            storedHeadingDegrees = storedLocation.getHeading(AngleUnit.DEGREES);
        }
        else
        {
            if (goalPosition == Field.redGoal)
            {
                fieldCentricOffset = 90;
            }
            else
            {
                fieldCentricOffset = -90;
            }
        }
        hw.pinpoint.setPosition(storedLocation);
    }

    @Override
    public void loop()
    {
        // Update Methods
        hw.updateTeleOp();
        hw.drivetrain.fieldcentricDrive(this, (hw.pinpoint.getPosition().getHeading(AngleUnit.RADIANS) - Math.toRadians(storedHeadingDegrees)) + Math.toRadians(fieldCentricOffset));
        if (velocityAdjustmentRuntime.seconds() > (1.0 / 50))
        {
            hw.turret.updateFlywheelAndHood(hw.pinpoint.getPosition(), goalPosition);
            velocityAdjustmentRuntime.reset();
        }

        if (isTargeting)
        {
            // if targeting is on, update the turret with the new target
            hw.turret.setTarget(hw.pinpoint.getPosition(), goalPosition);
        }

        if (gamepad1.yWasPressed())
        {
            // Switch Light Mode from solid to flashing, or from flashing to solid
            hw.lights.setLightMode(((hw.lights.getLightMode() == RGBLightController.LEDMode.SOLID) ? RGBLightController.LEDMode.FLASH : RGBLightController.LEDMode.SOLID));
            // Toggle targeting
            isTargeting = !isTargeting;
        }

        // Right Trigger (Firing)
        if (gamepad1.right_trigger > 0.2) {
            hw.intake.openGate();
        } else {
            hw.intake.closeGate();
        }
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.intake();
        }
        if (gamepad1.right_trigger < 0.5) {
            if (isIntaking) {
                hw.intake.intake();
            } else if (!isIntaking) {
                hw.intake.stop();
            }
        }

        if (gamepad1.aWasPressed())
        {
            isIntaking = !isIntaking;
        }

        if (gamepad1.xWasPressed())
        {
            hw.turret.ToggleFlywheel();
        }

        telemetry.addLine("Targeting: " + isTargeting);
        telemetry.addLine("Hood Target Position: " + String.format(Locale.US, "%.2f", hw.turret.getHoodTarget()));
        telemetry.addLine("Flywheel Target Velocity: " + String.format(Locale.US, "%.2f", hw.turret.getCurrentVelocity()));
        telemetry.addLine("Position: " + PoseUtils.poseToString(hw.pinpoint.getPosition(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(hw.pinpoint.getPosition(), goalPosition));
    }
}