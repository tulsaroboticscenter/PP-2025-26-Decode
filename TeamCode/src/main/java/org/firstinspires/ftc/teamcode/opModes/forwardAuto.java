package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;


@Autonomous(name = "Forward Auto", group = "Robot", preselectTeleOp = "SauronBlue")
public class forwardAuto extends LinearOpMode {

    private static final HWProfile robot = new HWProfile();
    private final MechOps ops = new MechOps(robot, this);
    private final Targeting target = new Targeting(robot, this);

    private final Pose2D goalPosition = new Pose2D(DistanceUnit.INCH, -15, 0, AngleUnit.DEGREES, 0);

    public void runOpMode() {
        robot.init(hardwareMap, false);
        robot.pinpoint.resetPosAndIMU();

        waitForStart();

        if (opModeIsActive()) {

            ops.setAllMotors(0.5);
            sleep(500);
            ops.allStop();

            telemetry.addData("Ending Position: ", robot.pinpoint.getPosX() + ", " + robot.pinpoint.getPosY());
            ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
            ops.writePose(goalPosition, "GoalPositionFile");
        }
    }



}
