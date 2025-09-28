package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;


@Autonomous(name = "AutoRed", group = "Robot", preselectTeleOp = "SauronRed")
public class autoRed extends LinearOpMode {

    private static final HWProfile robot = new HWProfile();
    private final MechOps ops = new MechOps(robot, this);
    private final Targeting target = new Targeting(robot, this);

    private final Pose2D goalPosition = new Pose2D(DistanceUnit.INCH, -15, 0, AngleUnit.DEGREES, 0);

    // temporary pose for going forward
    private final Pose2D forward = new Pose2D(DistanceUnit.INCH, 100, -100, AngleUnit.DEGREES, 0);



    public void runOpMode() {
        robot.init(hardwareMap, false);
        robot.pinpoint.resetPosAndIMU();

        waitForStart();

        if (opModeIsActive()) {

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
            robot.leftFrontDrive.setPower(-0.3);
            robot.leftBackDrive.setPower(-0.3);
            robot.rightFrontDrive.setPower(0.3);
            robot.rightBackDrive.setPower(0.3);
            sleep(700);
            ops.setAllMotors(0.3);
            sleep(2000);
            ops.allStop();
            sleep(2000);

            telemetry.addData("Ending Position: ", robot.pinpoint.getPosX() + ", " + robot.pinpoint.getPosY());
            ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
            ops.writePose(goalPosition, "GoalPositionFile");
        }
    }



}
