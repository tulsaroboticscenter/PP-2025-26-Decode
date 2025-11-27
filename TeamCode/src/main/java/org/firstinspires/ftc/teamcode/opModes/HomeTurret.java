package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@TeleOp(name="Home Turret", group="Robot")
//@Disabled
public class HomeTurret extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final Targeting targeting = new Targeting(robot, this);
    private final MechOps ops = new MechOps(robot, this);
    private final FieldMarkers markers = new FieldMarkers();

    private Pose2D goalPosition = markers.blueGoal;

    public static double NEW_P = 15;
    public static double NEW_I = 1;
    public static double NEW_D = 0.001;
    public static double NEW_F = 1;

    private HardwareMap hwMap;

    private boolean isSwitchFlipped = false;


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

        robot.turretRotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turretRotationMotor.setPower(0);

        /* Wait for the game driver to press play */
        waitForStart();

        telemetry.addData("Rotate Left", "D-pad Left");
        telemetry.addData("Rotate Right", "D-pad Right");
        telemetry.update();

        while(opModeIsActive()){

            if (gamepad1.dpad_left && !isSwitchFlipped) {
                robot.turretRotationMotor.setPower(0.5);
            } else if (gamepad1.dpad_right && !isSwitchFlipped) {
                robot.turretRotationMotor.setPower(-0.5);
            } else {
                robot.turretRotationMotor.setPower(0);
            }

            if (robot.turretLimitSwitch.isPressed()) {
                isSwitchFlipped = true;
                telemetry.addLine("Homed.");
                telemetry.update();
            }

            telemetry.addData("Limit Switch", robot.turretLimitSwitch.isPressed());
            telemetry.update();

        }
        robot.pinpoint.update();
        ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
    }
}