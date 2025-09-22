package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

public class HWProfile {

    public Pose2D goalPositionBlue = new Pose2D(DistanceUnit.MM, 100, 2692, AngleUnit.DEGREES, 0);

    public final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public final double LAUNCHER_TARGET_VELOCITY = 1300;
    public final double LAUNCHER_MIN_VELOCITY = 1200;

    // Declare OpMode members.
    public DcMotor leftFrontDrive = null; // driveLF
    public DcMotor rightFrontDrive = null; // driveRF
    public DcMotor leftBackDrive = null; // driveLR
    public DcMotor rightBackDrive = null; // driveRR
    public DcMotorEx launcher = null; // launcher
    public CRServo leftFeeder = null; // leftFeeder


    public GoBildaPinpointDriver pinpoint = null; // pinpoint


    public ElapsedTime feederTimer = new ElapsedTime();


    HardwareMap hwMap =  null;

    public HWProfile() {

    }
    public void init(HardwareMap ahwMap, boolean TeleOp) {

        hwMap = ahwMap;

        leftFrontDrive = hwMap.get(DcMotor.class, "driveLF");
        rightFrontDrive = hwMap.get(DcMotor.class, "driveRF");
        leftBackDrive = hwMap.get(DcMotor.class, "driveLR");
        rightBackDrive = hwMap.get(DcMotor.class, "driveRR");


        launcher = hwMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hwMap.get(CRServo.class, "leftFeeder");

        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");

        // reverse one side to make sure all motors are in sync
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);


        leftFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // pinpoint.setEncoderDirections();
    }
}
