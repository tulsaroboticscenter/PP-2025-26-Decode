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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="tele", group="Robot")
public class TeleOp extends OpMode
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
//        hw.pinpoint.setPosition(storedLocation);
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

        //hw.drivetrain.isTargeting = gamepad1.square;

        // Switch Light Mode from solid to flashing, or from flashing to solid
        if (gamepad1.triangleWasPressed())
        {
            if (!(totalRuntime.seconds() > 110))
            {
                hw.lights.setLightMode(((hw.lights.getLightMode() == RGBLightController.LEDMode.SOLID) ? RGBLightController.LEDMode.FLASH : RGBLightController.LEDMode.SOLID));
            }
            // Toggle targeting
            hw.turret.isTargeting = !hw.turret.isTargeting;
            hw.lights.setLightMode((hw.drivetrain.isTargeting) ? RGBLightController.LEDMode.FLASH : RGBLightController.LEDMode.SOLID);
        }

        if (isParking)
        {
            hw.turret.setTarget(hw.turret.HeadingToServoValue(0, AngleUnit.DEGREES));
            //hw.turret.manuallySetFlywheelAndHood(0, 0);
            hw.intake.stop();
        }
        else if (hw.turret.isTargeting && !hw.drivetrain.isTargeting)
        {
            // if targeting is on, update the turret with the new target
            hw.turret.setTarget(pos, goalPosition);
        }
        else
        {
            hw.turret.setTarget(0, AngleUnit.DEGREES);
        }

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

        if (gamepad1.left_bumper)
        {
            hw.drivetrain.slowDown();
        }
        else
        {
            hw.drivetrain.speedUp();
        }


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

        if (gamepad1.optionsWasPressed())
        {
            hw.pinpoint.setPosition((startingSide == Field.Side.BLUE) ? Field.blueHumanPlayerZone : Field.redHumanPlayerZone);
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

        ptelemetry.setUpdateInterval(50);

        ptelemetry.addData("Target Flywheel Velocity", hw.turret.targetVelocity);
        ptelemetry.addData("Average Flywheel Velocity", hw.turret.getAverageFlywheelVelocity());
        ptelemetry.addData("Left Flywheel Motor Velocity", hw.turret.launcherL.getVelocity());
        ptelemetry.addData("Right Flywheel Motor Velocity", hw.turret.launcherR.getVelocity());

        ptelemetry.addData("Left Flywheel Current Draw (Amps)", hw.turret.launcherL.getCurrent(CurrentUnit.AMPS));
        ptelemetry.addData("Right Flywheel Current Draw (Amps)", hw.turret.launcherR.getCurrent(CurrentUnit.AMPS));

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
        ptelemetry.addLine("--- current draw ---");
        ptelemetry.addData("Drivetrain (A)",         String.format(Locale.US, "%.2f", driveCurrentTotal));
        ptelemetry.addData("Intake (A)",             String.format(Locale.US, "%.2f", intakeCurrent));
        ptelemetry.addData("Turret (A)",             String.format(Locale.US, "%.2f", turretCurrentTotal));
        ptelemetry.addData("TOTAL (A)",              String.format(Locale.US, "%.2f", totalRobotCurrent));

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

        telemetry.addLine("--- current draw ---");
        telemetry.addData("Drivetrain (A)",         String.format(Locale.US, "%.2f", driveCurrentTotal));
        telemetry.addData("Intake (A)",             String.format(Locale.US, "%.2f", intakeCurrent));
        telemetry.addData("Turret (A)",             String.format(Locale.US, "%.2f", turretCurrentTotal));
        telemetry.addData("TOTAL (A)",              String.format(Locale.US, "%.2f", totalRobotCurrent));

        telemetry.update();
    }
}