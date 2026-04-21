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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="turretTest", group="Robot")
public class turretTest extends OpMode
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


    private boolean isTargeting = false;
    boolean isIntaking = false;
    boolean isParking = false;
    double storedHeadingDegrees = 90.0;
    double botHeading = 0.0;
    Pose2D storedLocation;
    boolean loaded = false;
    Pose2D startingPosition = null;

    double lastHeading = 0;
    double continuousHeading = 0;

    Field.Side startingSide = null;

    enum parkStatus {
        NOT_PARKED,
        MOBILE_PARKED,
        FULL_PARKED
    }

    parkStatus ParkStatus = parkStatus.NOT_PARKED;

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
            goalPosition = Field.redGoal;
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
        telemetry.update();
    }

    boolean testing = false;
    Field.Side testingSide = Field.Side.RED;

    @Override
    public void init_loop()
    {
        hw.updateInitTeleOp();
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
        hw.turret.spinUpFlywheel();
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
            //goalPosition = Field.redGoal;
        }
        hw.pinpoint.setPosition(storedLocation);
        lastHeading = storedLocation.getHeading(AngleUnit.DEGREES);
    }

    private Pose2D pos;
    @Override
    public void loop()
    {
        pos = hw.pinpoint.getPosition();
        // Update Methods
        hw.updateTeleOp(this);
        hw.turret.updateFlywheelAndHood(pos, goalPosition);

        hw.drivetrain.fieldOrientedDrive(this, pos, goalPosition, storedLocation.getHeading(AngleUnit.RADIANS), startingSide);

        // Right Trigger (Firing)
        if (gamepad1.right_trigger > 0.5)
        {
            hw.intake.isForceIntaking = true;
            hw.intake.openGate();
        }
        else if (gamepad1.right_trigger < 0.5)
        {
            hw.intake.isForceIntaking = false;
            hw.intake.closeGate();
        }

        if (gamepad1.aWasPressed())
        {
            hw.intake.toggle();
        }

        if (gamepad1.rightBumperWasPressed())
        {
            if (ParkStatus == parkStatus.NOT_PARKED)
            {
                isParking = true;
                hw.drivetrain.mobilePark();
                ParkStatus = parkStatus.MOBILE_PARKED;
            }
            else if (ParkStatus == parkStatus.MOBILE_PARKED)
            {
                hw.drivetrain.park();
                ParkStatus = parkStatus.FULL_PARKED;
            }
            else
            {
                isParking = false;
                hw.drivetrain.unpark();
                ParkStatus = parkStatus.NOT_PARKED;
            }
        }

        if (gamepad1.x)
        {
            hw.drivetrain.slowDown();
        }
        else
        {
            hw.drivetrain.speedUp();
        }

        if (gamepad1.dpadLeftWasPressed())
        {
            hw.turret.setTurretPosition(0);
        }
        else if (gamepad1.dpadRightWasPressed())
        {
            hw.turret.setTurretPosition(1);
        }

        if (gamepad1.optionsWasPressed())
        {
            hw.pinpoint.pinpoint.setPosition((startingSide == Field.Side.BLUE) ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
        }

        if (totalRuntime.seconds() > 100 && !endgame)
        {
            endgame = true;
            hw.lights.setLightColor(1);
            gamepad1.setLedColor(1, 1, 1, 1000000000);
            gamepad1.rumbleBlips(4);
        }
        if (totalRuntime.seconds() > 110 && hw.lights.getLightColor() != 0.504)
        {
            hw.lights.setLightMode(RGBLightController.LEDMode.PULSE);
        }

        if (gamepad1.psWasPressed())
        {
            hw.turret.toggleFlywheel();
        }

        ptelemetry.setUpdateInterval(50);

        //ptelemetry.addData("Target Flywheel Velocity", hw.turret.velocity);
        //ptelemetry.addData("Average Flywheel Velocity", hw.turret.getAverageFlywheelVelocity());
        //ptelemetry.addData("Left Flywheel Motor Velocity", hw.turret.launcherL.getVelocity());
        //ptelemetry.addData("Right Flywheel Motor Velocity", hw.turret.launcherR.getVelocity());

//        ptelemetry.addData("Left Flywheel Current Draw (Amps)", hw.turret.launcherL.getCurrent(CurrentUnit.AMPS));
//        ptelemetry.addData("Right Flywheel Current Draw (Amps)", hw.turret.launcherR.getCurrent(CurrentUnit.AMPS));

        ptelemetry.addLine("");

        ptelemetry.addData("LeftFront Current", hw.drivetrain.leftFront.getCurrent(CurrentUnit.AMPS));
        ptelemetry.addData("RightFront Current", hw.drivetrain.rightFront.getCurrent(CurrentUnit.AMPS));
        ptelemetry.addData("LeftBack Current", hw.drivetrain.leftBack.getCurrent(CurrentUnit.AMPS));
        ptelemetry.addData("RightBack Current", hw.drivetrain.rightBack.getCurrent(CurrentUnit.AMPS));

        //ptelemetry.addData("Hood Target", hw.turret.hoodTarget);

        ptelemetry.addLine("");

        ptelemetry.addData("Intake Inner Current", hw.intake.innerIntakeMotor.getCurrent(CurrentUnit.AMPS));
        ptelemetry.addData("Intake Outer Current", hw.intake.outerIntakeMotor.getCurrent(CurrentUnit.AMPS));

        ptelemetry.addLine("");

//        ptelemetry.addData("Shooter Left Current", hw.turret.launcherL.getCurrent(CurrentUnit.AMPS));
//        ptelemetry.addData("Shooter Right Current", hw.turret.launcherR.getCurrent(CurrentUnit.AMPS));

        ptelemetry.addData("Pinpoint Frequency", hw.pinpoint.pinpoint.getFrequency());
        ptelemetry.addData("X Encoder Raw", hw.pinpoint.pinpoint.getEncoderX());
        ptelemetry.addData("Y Encoder Raw", hw.pinpoint.pinpoint.getEncoderY());
        ptelemetry.addData("Heading", hw.pinpoint.pinpoint.getHeading());

        ptelemetry.update();
        telemetry.addLine("Drive being inputted?: " + hw.drivetrain.isInputtingOutsideDeadzone(gamepad1));
        telemetry.addLine("Continuous Heading: " + hw.turret.continuousHeading);
        telemetry.addLine("Targeting: " + isTargeting);
        telemetry.addLine("Parked: " + hw.drivetrain.isParked);
//        telemetry.addLine("Hood Target Position: " + String.format(Locale.US, "%.2f", hw.turret.getHoodTarget()));
//        telemetry.addLine("Flywheel Target Velocity: " + String.format(Locale.US, "%.2f", hw.turret.getCurrentVelocity()));
        telemetry.addLine("Position: " + PoseUtils.poseToString(pos, DistanceUnit.INCH, AngleUnit.DEGREES));
        //telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(pos, goalPosition));
        telemetry.addLine("Time Passed: " + String.format(Locale.US, "%.2f", totalRuntime.seconds()) + "s");
        telemetry.update();
    }
}