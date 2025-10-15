package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.FieldMarkers;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;

import java.util.Locale;

/** @noinspection ALL*/
@TeleOp(name="Test", group="Robot")
//@Disabled
public class Test extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final Targeting targeting = new Targeting(robot, this);
    private final MechOps ops = new MechOps(robot, this);
    private final FieldMarkers markers = new FieldMarkers();

    private Pose2D goalPosition = markers.redGoal;

    public static double NEW_P = 15;
    public static double NEW_I = 1;
    public static double NEW_D = 0.001;
    public static double NEW_F = 1;

    private HardwareMap hwMap;


    @Override
    public void runOpMode() {

        // INITIALIZATION OPERATIONS

        robot.init(hardwareMap, true);
        robot.pinpoint.update();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        double botHeading = 0.0;

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

        // timer for targeting PD-controller
        ElapsedTime pdTimer = new ElapsedTime();

        /* Wait for the game driver to press play */
        waitForStart();



        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime totalRuntime = new ElapsedTime();
        ElapsedTime targetingDelayRuntime = new ElapsedTime();
        ElapsedTime targetingRefreshRuntime = new ElapsedTime();
        ElapsedTime velocityAdjustmentRuntime = new ElapsedTime();

        totalRuntime.reset();
        targetingDelayRuntime.reset();
        targetingRefreshRuntime.reset();
        velocityAdjustmentRuntime.reset();
        pdTimer.reset();

        double velocity = robot.LAUNCHER_TARGET_VELOCITY;

        // booleans for keeping track of toggles
        boolean isTargeting = false;

        // doubles for movement
        double y = 0;
        double x = 0;
        double rx = 0;

        gamepad1.setLedColor(0, 1, 0, 100000000);
        gamepad1.rumbleBlips(3);

//        requestOpModeStop();

        /**

         MAIN LOOP

         **/

        while(opModeIsActive()){
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;


            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.1f in, Y: %.1f in, H: %.1f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data); // prints current positional data from pinpoint

            botHeading = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);



            if (isTargeting)
            {
                rx = targeting.getTargetingRotationPowerPD(pos, goalPosition, 1, pdTimer, true);
            }
            else
            {
                rx = gamepad1.right_stick_x;
            }



            if (gamepad1.a && targetingDelayRuntime.time() >= 0.4) {
                if (isTargeting)
                {
                    isTargeting = false;
                    gamepad1.setLedColor(0, 1, 0, 100000000);
                    gamepad1.rumble(300);
                }
                else if (!isTargeting)
                {
                    isTargeting = true;
                    gamepad1.setLedColor(1, 0, 0, 100000000);
                    gamepad1.rumbleBlips(2);
                }
                targetingDelayRuntime.reset();
            }

            if (gamepad1.share) {
                robot.pinpoint.resetPosAndIMU();
            }

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

            telemetry.addData("Targeting", (isTargeting) ? "On" : "Off");
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Time Total", totalRuntime.time());
            telemetry.update();
        }
        robot.pinpoint.update();
        ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
    }
}