package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

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



    boolean isTargeting = false;
    boolean isIntaking = false;
    double storedHeading = 90.0;
    double botHeading = 0.0;
    Pose2D storedLocation;
    boolean loaded = false;
    Pose2D startingPosition = null;

    @Override
    public void init()
    {
        hw.initTeleOp(hardwareMap);
        if (Field.lastKnownPosition != null)
        {
            storedLocation = Field.lastKnownPosition;
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
        storedHeading = storedLocation.getHeading(AngleUnit.DEGREES);

        hw.lights.setLightMode(RGBLightController.LEDMode.WAKE);

        totalRuntime.reset();
        targetingDelayRuntime.reset();
        targetingRefreshRuntime.reset();
        velocityAdjustmentRuntime.reset();
        robotLeadRuntime.reset();
        intakeToggleRuntime.reset();
        hoodToggleRuntime.reset();
        flywheelToggleRuntime.reset();
    }

    @Override
    public void init_loop()
    {
        hw.updateTeleOp();
    }

    @Override
    public void start()
    {
        hw.turret.initHood();
    }

    @Override
    public void loop()
    {
        hw.updateTeleOp();
        if (velocityAdjustmentRuntime.seconds() > (1.0 / 50))
        {
            hw.turret.updateFlywheelAndHood(hw.pinpoint.getPosition(), goalPosition);
            velocityAdjustmentRuntime.reset();
        }
        hw.drivetrain.fieldcentricDrive(this, hw.pinpoint.getPosition().getHeading(AngleUnit.DEGREES), storedHeading);

        if (isTargeting)
        {
            hw.turret.setTarget(hw.pinpoint.getPosition(), goalPosition);
        }

        if (gamepad1.yWasPressed())
        {
            if (isTargeting)
            {
                hw.lights.setLightMode(RGBLightController.LEDMode.SOLID);
            }
            else
            {
                hw.lights.setLightMode(RGBLightController.LEDMode.FLASH);
            }
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

        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(hw.pinpoint.getPosition(), goalPosition));
    }
}
