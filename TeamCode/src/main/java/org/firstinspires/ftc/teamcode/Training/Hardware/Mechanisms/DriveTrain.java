package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
    class containing the drive train data types and movement methods
 */
public class DriveTrain {
    double maxPower = 1.0;
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

    // tank drive method
    public void driveRobotTank(double fwdPower, double rotate) {

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

    // mechanum drive base - robot centric
    public void driveRobotMecanum(double fwdPower, double strafe, double rotate){


        double frontLeftPower = fwdPower + strafe + rotate;
        double backLeftPower = fwdPower - strafe + rotate;
        double frontRightPower = fwdPower - strafe - rotate;
        double backRightPower = fwdPower + strafe - rotate;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        leftFront.setPower(frontLeftPower / maxPower);
        leftBack.setPower(backLeftPower / maxPower);
        rightFront.setPower(frontRightPower / maxPower);
        rightBack.setPower(backRightPower / maxPower);
    }

    public void driveRobotField(double fwdPower, double strafe, double rotate, double curPosRadians ) {
        // First, convert direction being asked to drive to polar coordinates

        double theta = Math.atan2(fwdPower, strafe);
        double r = Math.hypot(strafe, fwdPower);

        // Second, rotate angle by the angle the robot is pointing

        theta = AngleUnit.normalizeRadians(theta - curPosRadians);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        driveRobotMecanum(newForward, newRight, rotate);
    }

}
