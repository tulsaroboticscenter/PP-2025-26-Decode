package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.FieldMarkers;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;

/** @noinspection ALL*/
@TeleOp(name="Home Hood", group="Robot")
//@Disabled
public class HomeHood extends LinearOpMode {


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

    private ElapsedTime hoodAdjustRuntime = new ElapsedTime();

    private double hoodPosition = 1;


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
        ops.setHoodPosition(1);

        telemetry.addData("Rotate Left", "D-pad Left");
        telemetry.addData("Rotate Right", "D-pad Right");
        telemetry.update();

        hoodAdjustRuntime.reset();

        /* Wait for the game driver to press play */
        waitForStart();



        while(opModeIsActive())
        {

            if (hoodAdjustRuntime.seconds() > 0.25)
            {
                if (gamepad1.dpad_down)
                {
                    hoodPosition = 0;
                }
                else if (gamepad1.dpad_up)
                {
                    hoodPosition = 1;
                }
                else if (gamepad1.left_stick_y > 0.2 || gamepad1.left_stick_y < -0.2)
                {
                    hoodPosition += (0.1 * gamepad1.left_stick_y);
                }
                hoodAdjustRuntime.reset();
            }

            if (hoodPosition > 1)
            {
                hoodPosition = 1;
            }
            else if (hoodPosition < 0)
            {
                hoodPosition = 0;
            }

            ops.setHoodPosition(hoodPosition);
            telemetry.addData("Hood Goal Position", hoodPosition);
            telemetry.addData("Left Hood Servo Position", robot.hoodServoL.getPosition());
            telemetry.addData("Right Hood Servo Position", robot.hoodServoR.getPosition());
            telemetry.addLine("------------------------------");
            telemetry.addLine("Set Position to 1: Dpad Up");
            telemetry.addLine("Set Position to 0: Dpad Down");
            telemetry.addLine("Adjust Hood Up/Down: Left Stick");
            telemetry.update();
        }
        
        robot.pinpoint.update();
        ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
    }
}