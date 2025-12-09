package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.Field;
import org.firstinspires.ftc.teamcode.Libraries.GamepadEffects;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.RGBLightController;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;
import org.firstinspires.ftc.teamcode.Libraries.TurretTargeting;
import org.firstinspires.ftc.teamcode.Libraries.Velocity;

import java.util.Locale;

/** @noinspection ALL*/
@TeleOp(name="TeleRedFar", group="Robot")
//@Disabled
public class SauronRedFar extends LinearOpMode {



    /**

        EXPERIMENTAL OPMODE

     Current Function: Testing lead logic.

     **/

    private final static HWProfile robot = new HWProfile();
    private final Targeting targeting = new Targeting(robot);
    private final MechOps ops = new MechOps(robot, this);
    private final TurretTargeting turretTargeting = new TurretTargeting(robot, targeting);

    private final Velocity robotVel = new Velocity(robot, targeting);

    private final GamepadEffects gamepadEffects = new GamepadEffects();

    private Pose2D goalPosition = Field.redGoal;

    public static double NEW_P = 15;
    public static double NEW_I = 1;
    public static double NEW_D = 0.001;
    public static double NEW_F = 1;

    private HardwareMap hwMap;

    private enum LauncherStatus {
        LOW,
        MEDIUM,
        HIGH
    }

    @Override
    public void runOpMode() {

        // INITIALIZATION OPERATIONS

        robot.init(hardwareMap, true);
        robot.pinpoint.update();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();


        // Send telemetry message to signify robot waiting
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        double storedHeading = 0.0;
        double botHeading = 0.0;
        Pose2D storedLocation;
        boolean load = true;
        Pose2D startingPosition = Field.redSmallZone;

        try {
            storedLocation = ops.readPose("PositionFile");
            String locationData = String.format(Locale.US, "X: %.1f in, Y: %.1f in, %.1f degrees", storedLocation.getX(DistanceUnit.INCH), storedLocation.getY(DistanceUnit.INCH), storedLocation.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position Found", locationData);
            telemetry.addLine("Load? (A: Yes (default), B: No)");
            telemetry.update();
            gamepad1.runLedEffect(gamepadEffects.wakeRed);
            ops.setRGB(0.28);
            ops.setRGBMode(RGBLightController.LEDMode.WAKE);
            while (opModeInInit()) {
                if (gamepad1.b) {
                    load = false;
                    telemetry.addLine("Position Disregarded.");
                    telemetry.addLine("Starting at Blue Side Small Zone on Start.");
                    telemetry.update();
                } else if (gamepad1.a || isStopRequested()) {
                    break;
                }
                ops.updateRGB();
            }


        } catch (Exception e) {
            telemetry.addLine("Error reading PoseFile");
            telemetry.addLine(e.toString());
            storedLocation = Field.blueSmallZone;
            storedHeading = startingPosition.getHeading(AngleUnit.DEGREES);;
        }

        telemetry.addData("Stored Heading from File (if any): ", storedHeading);
        telemetry.addData("Current Bot Heading: ", botHeading);
        telemetry.update();

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        // timer for targeting PD-controller
        ElapsedTime pdTimer = new ElapsedTime();

        /* Wait for the game driver to press play */
        waitForStart();

        LauncherStatus status = LauncherStatus.LOW;
        ops.setRGB(0.5);

        if (load) {
            robot.pinpoint.setPosition(storedLocation);
            robot.pinpoint.update();
            storedHeading = storedLocation.getHeading(AngleUnit.DEGREES);
        } else if (!load) {
            robot.pinpoint.setPosition(startingPosition);
            robot.pinpoint.update();
            storedHeading = startingPosition.getHeading(AngleUnit.DEGREES);
        }

        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime totalRuntime = new ElapsedTime();
        ElapsedTime targetingDelayRuntime = new ElapsedTime();
        ElapsedTime targetingRefreshRuntime = new ElapsedTime();
        ElapsedTime velocityAdjustmentRuntime = new ElapsedTime();
        ElapsedTime robotLeadRuntime = new ElapsedTime();
        ElapsedTime intakeToggleRuntime = new ElapsedTime();
        ElapsedTime hoodToggleRuntime = new ElapsedTime();
        ElapsedTime flywheelToggleRuntime = new ElapsedTime();

        totalRuntime.reset();
        targetingDelayRuntime.reset();
        targetingRefreshRuntime.reset();
        velocityAdjustmentRuntime.reset();
        pdTimer.reset();
        robotLeadRuntime.reset();
        intakeToggleRuntime.reset();
        hoodToggleRuntime.reset();
        flywheelToggleRuntime.reset();


        Pose2D leadPose = robotVel.getLeadTarget(goalPosition);

        double velocity = robot.LAUNCHER_LOW_VELOCITY;

        // booleans for keeping track of toggles
        boolean isTargeting = false;
        boolean spinning = false;
        boolean isIntaking = false;
        boolean isOutaking = false;
        boolean isHoodHigh = false;

        boolean endgame = false;

        double y = 0;
        double x = 0;
        double rx = 0;

        gamepad1.setLedColor(0, 1, 0, 100000000);
        gamepad1.rumbleBlips(3);

        robot.turretRotationMotor.setPower(1);

        ops.setLauncherVelocity(0);

//        requestOpModeStop();

        /**

         MAIN LOOP

         **/

        while(opModeIsActive())
        {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.1f in, Y: %.1f in, H: %.1f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data); // prints current positional data from pinpoint

            botHeading = Math.toRadians(pos.getHeading(AngleUnit.DEGREES) - storedHeading) + (Math.PI / 2);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (robotLeadRuntime.seconds() > 0.02) { // 50hz
                leadPose = robotVel.getLeadTarget(goalPosition);
            }


            // A (Toggle Targeting)
            if (gamepad1.a && targetingDelayRuntime.time() >= 0.6) {
                if (isTargeting)
                {
                    isTargeting = false;
                    gamepad1.rumble(50);
                    ops.setRGBMode(RGBLightController.LEDMode.SOLID);
                }
                else if (!isTargeting)
                {
                    isTargeting = true;
                    gamepad1.rumble(50);
                    ops.setRGBMode(RGBLightController.LEDMode.FLASH);
                }
                targetingDelayRuntime.reset();
            }


            // Right Trigger (Firing)
            if (gamepad1.right_trigger > 0.2) {
                robot.gateServo.setPosition(0.8);
            } else {
                robot.gateServo.setPosition(0.5);
            }
            if (gamepad1.right_trigger > 0.5) {
                robot.intakeMotor.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.right_trigger < 0.5) {
                if (isIntaking) {
                    robot.intakeMotor.setPower(1);
                } else if (!isIntaking) {
                    robot.intakeMotor.setPower(0);
                }
            }

            // B (Toggle Intake)
            if (gamepad1.b && intakeToggleRuntime.seconds() > 0.4)
            {
                if (isIntaking)
                {
                    isIntaking = false;
                }
                else if (!isIntaking)
                {
                    isIntaking = true;
                }
                intakeToggleRuntime.reset();
            }
            if (gamepad1.x && intakeToggleRuntime.seconds() > 0.4)
            {
                if (isOutaking)
                {
                    isOutaking = false;
                }
                else if (!isOutaking)
                {
                    isOutaking = true;
                }
                intakeToggleRuntime.reset();
            }

            if (gamepad1.right_bumper && hoodToggleRuntime.seconds() > 0.4)
            {
                if (status == LauncherStatus.LOW)
                {
                    ops.setHoodPosition(0.35);
                    ops.setRGB(0.388);
                    velocity = robot.LAUNCHER_MEDIUM_VELOCITY;
                    status = LauncherStatus.MEDIUM;
                }
                else if (status == LauncherStatus.MEDIUM)
                {
                    ops.setHoodPosition(0.6);
                    ops.setRGB(0.28);
                    velocity = robot.LAUNCHER_HIGH_VELOCITY;
                    status = LauncherStatus.HIGH;
                }
                else if (status == LauncherStatus.HIGH)
                {
                    ops.setHoodPosition(0);
                    ops.setRGB(0.5);
                    velocity = robot.LAUNCHER_LOW_VELOCITY;
                    status = LauncherStatus.LOW;
                }
                if (spinning) {
                    ops.setLauncherVelocity(velocity);
                }
                hoodToggleRuntime.reset();
            }



            if (gamepad1.dpad_down && velocityAdjustmentRuntime.seconds() > 0.4)
            {
                velocity -= 50;
                velocityAdjustmentRuntime.reset();
                if (spinning)
                {
                    ops.setLauncherVelocity(velocity);
                }
            }
            else if (gamepad1.dpad_up && velocityAdjustmentRuntime.seconds() > 0.4)
            {
                velocity += 50;
                velocityAdjustmentRuntime.reset();
                if (spinning)
                {
                    ops.setLauncherVelocity(velocity);
                }
            }

            if (gamepad1.x && flywheelToggleRuntime.seconds() > 0.4)
            {
                if (spinning)
                {
                    ops.setLauncherVelocity(0);
                    spinning = false;
                }
                else if (!spinning)
                {
                    ops.setLauncherVelocity(velocity);
                    spinning = true;
                }
                flywheelToggleRuntime.reset();
            }

            if (gamepad1.share) {
                robot.pinpoint.resetPosAndIMU();
            }

            if (isTargeting)
            {
                robot.turret.setTarget(pos, goalPosition);
            }

            if (endgame == false && totalRuntime.seconds() > 100)
            {
                endgame = true;
                gamepad1.setLedColor(1, 0, 0, 100000000);
                gamepad1.runLedEffect(gamepadEffects.wakeRed);
                gamepad1.rumbleBlips(5);
                ops.setRGB(1);
                ops.setRGBMode(RGBLightController.LEDMode.WAKE);
            }

            ops.updateRGB();
            robot.turret.update();

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFrontDrive.setPower(frontLeftPower);
            robot.leftBackDrive.setPower(backLeftPower);
            robot.rightFrontDrive.setPower(frontRightPower);
            robot.rightBackDrive.setPower(backRightPower);

            telemetry.addData("Target Flywheel Velocity", velocity);
            telemetry.addData("Right Launcher Motor Velocity: ", robot.launcherR.getVelocity());
            telemetry.addData("Left Launcher Motor Velocity", robot.launcherL.getVelocity());
            telemetry.addData("Targeting: ", isTargeting);
            telemetry.addData("Distance to Target (in): ", (targeting.getDistanceToTarget(pos, goalPosition) / 25.4));
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Robot Velocity", robotVel.getMagnitude());
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Time Total", totalRuntime.time());
            telemetry.update();
        }
        robot.pinpoint.update();
        ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
    }
}