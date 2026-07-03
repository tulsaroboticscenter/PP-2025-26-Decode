package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;


    public void init(HardwareMap hwMap){
        leftFront = hwMap.get(DcMotorEx.class, "driveLF");
        rightFront = hwMap.get(DcMotorEx.class, "driveRF");
        leftBack = hwMap.get(DcMotorEx.class, "driveLR");
        rightBack = hwMap.get(DcMotorEx.class, "driveRR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);

    }
    public void driveRobot(double fwdPower, double rotate) {
        double maxPower = 1.0;

        double frontLeftPower = fwdPower + rotate;
        double backLeftPower = fwdPower + rotate;
        double frontRightPower = fwdPower - rotate;
        double backRightPower = fwdPower - rotate;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        leftFront.setPower(frontLeftPower / maxPower);
        leftBack.setPower(backLeftPower / maxPower);
        rightFront.setPower(frontRightPower / maxPower);
        rightBack.setPower(backRightPower / maxPower);
    }


    }
