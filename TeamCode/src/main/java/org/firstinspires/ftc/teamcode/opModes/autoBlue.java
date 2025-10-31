package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.FieldMarkers;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

import java.io.PipedOutputStream;
import java.util.Locale;


@Autonomous(name = "AutoBlue", group = "Robot", preselectTeleOp = "SauronBlue")
public class autoBlue extends LinearOpMode {

    private static final HWProfile robot = new HWProfile();
    private final MechOps ops = new MechOps(robot, this);
    private final Targeting target = new Targeting(robot, this);
    private final FieldMarkers markers = new FieldMarkers();

    private Pose2D goalPosition = markers.blueGoal;

    enum States {
        SHOOT,
        PARK
    }

    States State = States.SHOOT;



    public void runOpMode() {
        robot.init(hardwareMap, false);
        while (robot.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            telemetry.addData("Status:", "Waiting on Pinpoint...");
            telemetry.update();
            robot.pinpoint.update();
        }
        robot.pinpoint.setPosition(markers.blueTouchingGoal);
        robot.pinpoint.update();
        sleep(300);
        String data = String.format(Locale.US, "{%.1f, %.1f} %.1f degrees", (robot.pinpoint.getPosX() / 25.4), (robot.pinpoint.getPosY() / 25.4), Math.toDegrees(robot.pinpoint.getHeading()));
        telemetry.addData("Starting Position: ", data);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            switch (State){
                case SHOOT:
                    robot.launcher.setVelocity(robot.LAUNCHER_TARGET_VELOCITY);
                    ops.setAllMotors(0.3);
                    sleep(1000);
                    ops.allStop();
                    sleep(200);

                    while (robot.launcher.getVelocity() < robot.LAUNCHER_TARGET_VELOCITY - 100) {
                        sleep(10);
                    }
                    robot.leftFeeder.setPower(robot.FULL_SPEED);
                    sleep(6000);
                    robot.leftFeeder.setPower(0);
                    robot.launcher.setVelocity(0);
                    sleep(200);
                    State = States.PARK;
                    break;

                case PARK:
                    robot.leftFrontDrive.setPower(0.3);
                    robot.leftBackDrive.setPower(0.3);
                    robot.rightFrontDrive.setPower(-0.3);
                    robot.rightBackDrive.setPower(-0.3);
                    sleep(700);
                    ops.setAllMotors(0.3);
                    sleep(2000);
                    ops.allStop();
                    sleep(2000);
                    break;
            }

            robot.pinpoint.update();

            telemetry.addData("Ending Position", "(" + robot.pinpoint.getPosX() + ", " + robot.pinpoint.getPosY() + ")");
            telemetry.update();
            ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
        }
    }
}
