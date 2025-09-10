package org.firstinspires.ftc.teamcode.Libraries;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class MechOps {

    HWProfile robot = new HWProfile();
    private double leftPower;
    private double rightPower;
    private double strafePower;

    public void StrafeDrive(double drive, double turn, double strafe) {

        leftPower    = -Range.clip(drive, -robot.FULL_SPEED, robot.FULL_SPEED);
        rightPower   = -Range.clip(drive, -robot.FULL_SPEED, robot.FULL_SPEED);
        strafePower = Range.clip(-strafe, -robot.FULL_SPEED, robot.FULL_SPEED);

        robot.leftFrontDrive.setPower(leftPower - turn + strafePower);
        robot.leftBackDrive.setPower(leftPower - turn - strafePower);
        robot.rightFrontDrive.setPower(rightPower + turn - strafePower);
        robot.rightBackDrive.setPower(rightPower + turn + strafePower);

    }

    public void launch() {
        if (robot.launcher.getVelocity() >= robot.LAUNCHER_MIN_VELOCITY) {
            robot.leftFeeder.setPower(robot.FULL_SPEED);
            robot.rightFeeder.setPower(robot.FULL_SPEED);
        }
    }
    public void stopLaunch() {
        robot.leftFeeder.setPower(robot.STOP_SPEED);
        robot.rightFeeder.setPower(robot.STOP_SPEED);
    }
}
