package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;

import java.io.File;

public class Drivetrain
{
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    public Servo park1 = null;
    public Servo park2 = null;

    public void init(HardwareMap hwMap)
    {
        leftFront = hwMap.get(DcMotorEx.class, "driveLF");
        rightFront = hwMap.get(DcMotorEx.class, "driveRF");
        leftBack = hwMap.get(DcMotorEx.class, "driveLR");
        rightBack = hwMap.get(DcMotorEx.class, "driveRR");

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

        park1 = hwMap.get(Servo.class, "park1");
        park2 = hwMap.get(Servo.class, "park2");

        park1.setPosition(0 + startingParkPosition);
        park2.setPosition(1 - startingParkPosition);
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

    public boolean parked = false;

    public double parkRange = 0.5;
    public double startingParkPosition = 0.05;

    double offset = 0;

    double heading = 0;

    public double drivePower = 1;

    public static double SLOW_DRIVING_SPEED = 0.75;

    public void slowDown()
    {
        drivePower = SLOW_DRIVING_SPEED;
    }
    public void speedUp()
    {
        drivePower = 1;
    }

    public void robotCentricDrive(double forward, double strafe, double rotate)
    {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        leftFront.setPower(drivePower * (frontLeftPower / maxPower));
        leftBack.setPower(drivePower * (backLeftPower / maxPower));
        rightFront.setPower(drivePower * (frontRightPower / maxPower));
        rightBack.setPower(drivePower * (backRightPower / maxPower));
    }

    public void fieldcentricDrive(OpMode opmode, double botHeadingRadians, double startingHeadingRadians, Field.Side side)
    {
        Y = -opmode.gamepad1.left_stick_y;
        X = opmode.gamepad1.left_stick_x;
        rX = opmode.gamepad1.right_stick_x;

        heading = botHeadingRadians - startingHeadingRadians;

        offset = ((side == Field.Side.RED) ? Math.toRadians(0) : Math.toRadians(180));

        rotX = X * Math.cos(heading + offset) - Y * Math.sin(heading + offset);
        rotY = X * Math.sin(heading + offset) + Y * Math.cos(heading + offset);

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

    public void fieldOrientedDrive(OpMode opmode, double currentHeadingRadians, double storedHeadingRadians, Field.Side side)
    {
        double forward = -opmode.gamepad1.left_stick_y;
        double strafe = opmode.gamepad1.left_stick_x;
        double rotate = opmode.gamepad1.right_stick_x;

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        offset = ((side == Field.Side.RED) ? Math.toRadians(0) : Math.toRadians(179.9));
        theta = AngleUnit.normalizeRadians((theta + offset) - currentHeadingRadians);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);


        robotCentricDrive(newForward, newStrafe, rotate);
    }
    public void fieldOrientedDrive(OpMode opmode, Pose2D currentPosition, double storedHeadingRadians, Field.Side side)
    {
        double forward = -opmode.gamepad1.left_stick_y;
        double strafe = opmode.gamepad1.left_stick_x;
        double rotate = opmode.gamepad1.right_stick_x;

        double currentHeadingRadians = currentPosition.getHeading(AngleUnit.RADIANS);

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        offset = ((side == Field.Side.RED) ? Math.toRadians(0) : Math.toRadians(179.9));
        theta = AngleUnit.normalizeRadians((theta + offset) - currentHeadingRadians);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        robotCentricDrive(newForward, newStrafe, rotate);
    }

    public void playerCentricDrive(OpMode opmode, Pose2D currentPosition, Field.Side side)
    {
        double forward = -opmode.gamepad1.left_stick_y;
        double strafe = opmode.gamepad1.left_stick_x;
        double rotate = opmode.gamepad1.right_stick_x;

        double currentHeadingRadians = currentPosition.getHeading(AngleUnit.RADIANS);

        Pose2D playerLocation = ((side == Field.Side.RED) ? Field.redPlayer: Field.bluePlayer);

        double deltaX = currentPosition.getX(DistanceUnit.INCH) - playerLocation.getX(DistanceUnit.INCH);
        double deltaY = currentPosition.getY(DistanceUnit.INCH) - playerLocation.getY(DistanceUnit.INCH);

        double playerTheta = Math.atan2(deltaY, deltaX);

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        offset = ((side == Field.Side.RED) ? Math.toRadians(0) : Math.toRadians(179.9));
        theta = AngleUnit.normalizeRadians(((theta + offset) + playerTheta) - currentHeadingRadians);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        robotCentricDrive(newForward, newStrafe, rotate);
    }

    public void park()
    {
        parked = true;
        park1.setPosition((0 + startingParkPosition) + parkRange);
        park2.setPosition((1 - startingParkPosition) - parkRange);
    }
    public void unpark()
    {
        parked = false;
        park1.setPosition(0 + startingParkPosition);
        park2.setPosition(1 - startingParkPosition);
    }

    public void togglePark()
    {
        if (parked)
        {
            park1.setPosition(0 + startingParkPosition);
            park2.setPosition(1 - startingParkPosition);
        }
        else
        {
            park1.setPosition((0 + startingParkPosition) + parkRange);
            park2.setPosition((1 - startingParkPosition) - parkRange);
        }
        parked = !parked;
    }


    /**
     *
     * @return returns array of amps [leftFront, rightFront, leftRear, rightRear]
     */
    public double[] getCurrentAmps()
    {
        double[] amperages = {leftFront.getCurrent(CurrentUnit.AMPS), rightFront.getCurrent(CurrentUnit.AMPS), leftBack.getCurrent(CurrentUnit.AMPS), rightBack.getCurrent(CurrentUnit.AMPS)};
        return amperages;
    }
}
