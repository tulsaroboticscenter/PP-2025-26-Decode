package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret.getDegreesToTarget;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.Rev9AxisImu;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PIDFController;

@Configurable
public class Drivetrain
{
    // --------
    // HARDWARE
    // --------

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;

    public Servo park1 = null;
    public Servo park2 = null;

    // ---------------
    // DRIVE VARIABLES
    // ---------------

    public Follower follower; // May be used in tandem with Pedro Pathing later.

    // Parking values
    public double parkRange = 0.5;
    public double mobileParkRange = 0.35;
    public double startingParkPosition = 0.05;

    // Switches
    public boolean isParked = false;
    public boolean isTargeting = false;

    // Field-Centric
    double offset = 0;

    // Speed
    public static double drivePower = 1;
    public final double SLOW_DRIVING_SPEED = 0.5;

    // Input Detection
    public final double JOYSTICK_DEADZONE = 0.05;

    // Drivetrain Targeting PIDF Values (Tunable in Panels)
    @Sorter(sort = 1)
    public static double KpVal = 0.008;
    @Sorter(sort = 2)
    public static double KiVal = 0.0;
    @Sorter(sort = 3)
    public static double KdVal = 0.0004;
    @Sorter(sort = 4)
    public static double KfVal = 0.0;

    PIDFController rotationPID = new PIDFController(KpVal, KiVal, KdVal, KfVal, -1, 1);

    // --------------
    // INITIALIZATION
    // --------------

    public void init(HardwareMap hwMap)
    {
        leftFront = hwMap.get(DcMotorEx.class, "driveLF");
        if (leftFront == null)
        {
            throw new IllegalStateException("leftFront motor not found in hardware map. Check robot configuration.");
        }

        rightFront = hwMap.get(DcMotorEx.class, "driveRF");
        if (rightFront == null)
        {
            throw new IllegalStateException("rightFront motor not found in hardware map. Check robot configuration.");
        }

        leftBack = hwMap.get(DcMotorEx.class, "driveLR");
        if (leftBack == null)
        {
            throw new IllegalStateException("leftBack motor not found in hardware map. Check robot configuration.");
        }

        rightBack = hwMap.get(DcMotorEx.class, "driveRR");
        if (rightBack == null)
        {
            throw new IllegalStateException("rightBack motor not found in hardware map. Check robot configuration.");
        }

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);

        park1 = hwMap.get(Servo.class, "park1");
        if (park1 == null)
        {
            throw new IllegalStateException("park1 servo not found in hardware map. Check robot configuration.");
        }

        park2 = hwMap.get(Servo.class, "park2");
        if (park2 == null)
        {
            throw new IllegalStateException("park2 servo not found in hardware map. Check robot configuration.");
        }

        park1.setPosition(0 + startingParkPosition);
        park2.setPosition(1 - startingParkPosition);

        rotationPID.setTarget(0);
        rotationPID.setTolerance(1.0);

        //follower = Constants.createFollower(hwMap);
    }

    // -------------
    // DRIVE METHODS
    // -------------

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

    public void fieldOrientedDrive(OpMode opmode, Pose2D currentPosition, Pose2D targetPosition, double storedHeadingRadians, Field.Side side)
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

        rotationPID.setTarget(Math.atan2(targetPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH), targetPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH)));

        if (isTargeting)
            robotCentricDrive(newForward, newStrafe, rotationPID.calculate(currentHeadingRadians));
        else
            robotCentricDrive(newForward, newStrafe, rotate);
    }

    /**
     *   Recieves opmode, current position, and the side of the field the robot is on and creates a driver-centric movement system.
     *   Very experimental.
     */
    public void playerCentricDrive(OpMode opmode, Pose2D currentPosition, Field.Side side) // Very experimental.
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
        isParked = true;
        park1.setPosition((0 + startingParkPosition) + parkRange);
        park2.setPosition((1 - startingParkPosition) - parkRange);
    }
    public void mobilePark()
    {
        isParked = true;
        park1.setPosition((0 + startingParkPosition) + mobileParkRange);
        park2.setPosition((1 - startingParkPosition) - mobileParkRange);
    }
    public void unpark()
    {
        isParked = false;
        park1.setPosition(0 + startingParkPosition);
        park2.setPosition(1 - startingParkPosition);
    }

    public void slowDown()
    {
        drivePower = SLOW_DRIVING_SPEED;
    }
    public void speedUp()
    {
        drivePower = 1;
    }

    // -----------------
    // TELEMETRY METHODS
    // -----------------

    /**
     *
     * @return returns total current draw of the drivetrain motors in AMPS.
     */
    public double getCurrentAmps()
    {
        return leftFront.getCurrent(CurrentUnit.AMPS) + leftBack.getCurrent(CurrentUnit.AMPS) + rightBack.getCurrent(CurrentUnit.AMPS) + rightFront.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isInputtingOutsideDeadzone(Gamepad gamepad)
    {
        double leftY = -gamepad.left_stick_y;
        double leftX = gamepad.left_stick_x;
        double rightX = gamepad.right_stick_x;

        double r = Math.hypot(leftY, leftX);
        return r > JOYSTICK_DEADZONE || Math.abs(rightX) > JOYSTICK_DEADZONE;
    }
}
