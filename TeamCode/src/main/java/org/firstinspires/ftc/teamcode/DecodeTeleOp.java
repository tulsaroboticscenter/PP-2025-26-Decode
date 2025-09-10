package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;

@TeleOp(name = "DecodeTeleOp", group = "robot")
//@Disabled
public class DecodeTeleOp extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private final static MechOps ops = new MechOps();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);

        waitForStart();
        while(opModeIsActive()) {
            ops.StrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            // Y spins up launching motor, which then enables launching.
            if (gamepad1.y) {
                robot.launcher.setVelocity(robot.LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.b) { // stop flywheel
                robot.launcher.setVelocity(robot.STOP_SPEED);
            }

            if (gamepad1.right_trigger > 0.5) {
                // robot will not launch an artifact until the launching motor's velocity
                // meets or exceeds HWProfile.LAUNCHER_MINIMUM_VELOCITY
                ops.launch();
                robot.feederTimer.reset();
            }

            if (robot.feederTimer.seconds() > robot.FEED_TIME_SECONDS) {
                ops.stopLaunch();
            }

            telemetry.addData("Drive/Strafe: ", "Left Stick");
            telemetry.addData("Rotate: ", "Right Stick");
            telemetry.addData("Spin Up Launcher: ", "Y / Triangle");
            telemetry.addData("Spin Down Launcher: ", "B / Circle");
            telemetry.addData("Fire (Only available when spun up): ", "RT / R3");
            telemetry.addData("launcher velocity: ", robot.launcher.getVelocity());
            telemetry.addData("feed timer (in seconds): ", robot.feederTimer.seconds());
            telemetry.update();
        }
    }
}