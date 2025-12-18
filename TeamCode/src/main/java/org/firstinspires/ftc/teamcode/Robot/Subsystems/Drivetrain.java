package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain
{
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    public void init(HardwareMap hwMap)
    {
        leftFront = hwMap.get(DcMotor.class, "driveLF");
        rightFront = hwMap.get(DcMotor.class, "driveRF");
        leftBack = hwMap.get(DcMotor.class, "driveLR");
        rightBack = hwMap.get(DcMotor.class, "driveRR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);
    }

    double Y = 0;
    double X = 0;
    double rX = 0;
    double rotX = 0;
    double rotY = 0;
    double denominator = 0;
    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;

    public void fieldcentricDrive(OpMode opmode, double botHeading)
    {
        Y = -opmode.gamepad1.left_stick_y;
        X = opmode.gamepad1.left_stick_x;
        rX = opmode.gamepad1.right_stick_x;

        rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
        rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rX), 1);
        frontLeftPower = (rotY + rotX + rX) / denominator;
        backLeftPower = (rotY - rotX + rX) / denominator;
        frontRightPower = (rotY - rotX - rX) / denominator;
        backRightPower = (rotY + rotX - rX) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
}
