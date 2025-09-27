package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;

@Autonomous(name = "Auto1", group = "Robot", preselectTeleOp = "Sauron")
public class auto extends LinearOpMode {

    HWProfile robot = new HWProfile();
    MechOps ops = new MechOps();
    Targeting target = new Targeting(robot, this);



    public void runOpMode() {
        waitForStart();

        if (opModeIsActive()) {

            ops.setAllMotors(0.5);
            sleep(1000);
            ops.allStop();
            sleep(200);
            target.rotateToTarget(robot.goalPositionBlue, 2);
            sleep(200);
            robot.launcher.setVelocity(robot.LAUNCHER_TARGET_VELOCITY);
            while (robot.launcher.getVelocity() < robot.LAUNCHER_TARGET_VELOCITY - 100) {
                sleep(10);
            }
            robot.leftFeeder.setPower(robot.FULL_SPEED);
            sleep(6000);
            robot.leftFeeder.setPower(0);
            robot.launcher.setVelocity(0);

            ops.allStop();
            ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
        }
    }

}
