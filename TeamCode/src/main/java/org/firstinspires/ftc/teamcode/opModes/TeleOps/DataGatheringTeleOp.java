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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Gather Data", group="Robot")
public class DataGatheringTeleOp extends OpMode
{
    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private Field.Side side = null;
    private HardwareManager hw = new HardwareManager(hardwareMap);
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

    Field.Side startingSide = null;

    boolean endgame = false;

    @Override
    public void init()
    {
        hw.initTeleOp(hardwareMap);
        if (Field.lastKnownPosition != null)
        {
            if (Field.lastAllianceSide == Field.Side.BLUE)
            {
                hw.lights.setLightColor(RGBLightController.BLUE);
                goalPosition = Field.blueGoal;
                telemetry.addLine("Blue Side.");
            }
            else if (Field.lastAllianceSide == Field.Side.RED)
            {
                hw.lights.setLightColor(RGBLightController.RED);
                goalPosition = Field.redGoal;
                telemetry.addLine("Red Side.");
            }
            else
            {
                hw.lights.setLightColor(RGBLightController.RED);
                goalPosition = Field.redGoal;
                telemetry.addLine("Last alliance side not found. Defaulting to Red.");
            }
            storedLocation = Field.lastKnownPosition;
            loaded = true;
            startingSide = Field.lastAllianceSide;
            telemetry.addLine("Last known position found: " + storedLocation.getX(DistanceUnit.INCH) + ", " + storedLocation.getY(DistanceUnit.INCH) + ", " + storedLocation.getHeading(AngleUnit.DEGREES));
        }
        else
        {
            storedLocation = Field.redSmallZone;
            startingSide = Field.Side.RED;
            hw.lights.setLightColor(RGBLightController.RED);
            telemetry.addLine("Last known position not found. Defaulting to Red.");
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
                hw.lights.setLightColor((testingSide == Field.Side.RED) ? RGBLightController.BLUE : RGBLightController.RED);
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
            }
            else
            {
                storedLocation = Field.blueSmallZone;
                goalPosition = Field.blueGoal;
            }
            startingSide = ((testingSide == Field.Side.RED) ? Field.Side.RED : Field.Side.BLUE);
            storedHeadingDegrees = storedLocation.getHeading(AngleUnit.DEGREES);
        }
        else
        {
            goalPosition = Field.redGoal;
        }
        hw.pinpoint.setPosition(storedLocation);
    }

    private Pose2D pos;
    @Override
    public void loop()
    {
        pos = hw.pinpoint.getPosition();
        // Update Methods
        hw.updateTeleOp();

        //hw.drivetrain.fieldcentricDrive(this, (pos.getHeading(AngleUnit.RADIANS) - Math.toRadians(storedHeadingDegrees)) + Math.toRadians(fieldCentricOffset));
        //hw.drivetrain.fieldcentricDrive(this, pos.getHeading(AngleUnit.RADIANS), Math.toRadians(storedHeadingDegrees), startingSide);
       // hw.drivetrain.StrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        hw.drivetrain.fieldOrientedDrive(this, pos, storedLocation.getHeading(AngleUnit.RADIANS), startingSide);
        hw.lights.setLightColor(((hw.limelight.seesTarget()) ? RGBLightController.GREEN : RGBLightController.RED));

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

        if (gamepad1.optionsWasPressed())
        {
            hw.pinpoint.resetPosition(startingSide);
        }

        if (gamepad1.shareWasPressed())
        {
            hw.pinpoint.setPosition(Field.center);
        }

        if (gamepad1.dpadRightWasPressed())
        {
            hw.turret.incrementHoodTarget(0.05);
        }
        if (gamepad1.dpadLeftWasPressed())
        {
            hw.turret.incrementHoodTarget(-0.05);
        }

        if (velocityAdjustmentRuntime.seconds() > 0.2)
        {
            if (gamepad1.dpad_up)
            {
                hw.turret.incrementVelocity(50.0);
                velocityAdjustmentRuntime.reset();
            }
            else if (gamepad1.dpad_down)
            {
                hw.turret.incrementVelocity(-50.0);
                velocityAdjustmentRuntime.reset();
            }
        }

        ptelemetry.addData("Target Flywheel Velocity", hw.turret.velocity);
        ptelemetry.addData("Left Flywheel Motor Velocity", hw.turret.launcherL.getVelocity());
        ptelemetry.addData("Right Flywheel Motor Velocity", hw.turret.launcherR.getVelocity());

        ptelemetry.addData("Left Flywheel Current Draw (Amps)", hw.turret.launcherL.getCurrent(CurrentUnit.AMPS));
        ptelemetry.addData("Right Flywheel Current Draw (Amps)", hw.turret.launcherR.getCurrent(CurrentUnit.AMPS));

        ptelemetry.update();

        telemetry.addData("Target Flywheel Velocity", hw.turret.velocity);
        telemetry.addData("Left Flywheel Motor Velocity", hw.turret.launcherL.getVelocity());
        telemetry.addData("Right Flywheel Motor Velocity", hw.turret.launcherR.getVelocity());

        telemetry.addLine("Hood Target Position: " + String.format(Locale.US, "%.2f", hw.turret.getHoodTarget()));
        telemetry.addLine("Flywheel Target Velocity: " + String.format(Locale.US, "%.2f", hw.turret.getCurrentVelocity()));
        telemetry.addLine();
        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(pos, goalPosition));
        telemetry.addLine("Position" + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.update();
    }
}