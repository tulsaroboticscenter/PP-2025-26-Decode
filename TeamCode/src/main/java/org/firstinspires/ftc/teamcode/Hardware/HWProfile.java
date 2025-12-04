package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Libraries.RGBLightController;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

@Config
public class HWProfile {

    public final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public final double FULL_SPEED = 1.0;

    public final double LAUNCHER_LOW_VELOCITY = 1400;

    public final double LAUNCHER_MEDIUM_VELOCITY = 1800;
    public final double LAUNCHER_HIGH_VELOCITY = 2000;

    // Declare OpMode members.
    public DcMotor leftFrontDrive = null; // driveLF
    public DcMotor rightFrontDrive = null; // driveRF
    public DcMotor leftBackDrive = null; // driveLR
    public DcMotor rightBackDrive = null; // driveRR
    public DcMotorEx launcherR = null; // launcherR
    public DcMotorEx launcherL = null; // launcherL
    public DcMotor intakeMotor = null; // intakeMotor
    public DcMotor turretRotationMotor = null; // trMotor

    public Servo gateServo = null; // gate

    public Servo hoodServoL = null; //hoodServoL
    public Servo hoodServoR = null; //hoodServoR

    public Servo RGBLight1 = null;
    public Servo RGBLight2 = null;

    public RGBLightController light1 = null;
    public RGBLightController light2 = null;

    public RevTouchSensor turretLimitSwitch = null; // turretLimitSwitch


    public GoBildaPinpointDriver pinpoint = null; // pinpoint

    public Limelight3A limelight = null; // limelight

    public ElapsedTime feederTimer = new ElapsedTime();
    public ElapsedTime pdTimer = new ElapsedTime();

    HardwareMap hwMap =  null;

    public HWProfile() {

    }
    public void init(HardwareMap ahwMap, boolean TeleOp) {

        hwMap = ahwMap;

        light1 = new RGBLightController(RGBLight1);
        light2 = new RGBLightController(RGBLight2);

        leftFrontDrive = hwMap.get(DcMotor.class, "driveLF");
        rightFrontDrive = hwMap.get(DcMotor.class, "driveRF");
        leftBackDrive = hwMap.get(DcMotor.class, "driveLR");
        rightBackDrive = hwMap.get(DcMotor.class, "driveRR");

        launcherR = hwMap.get(DcMotorEx.class, "launcherR");
        launcherL = hwMap.get(DcMotorEx.class, "launcherL");

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        turretRotationMotor = hwMap.get(DcMotor.class, "turretRotationMotor");

        hoodServoL = hwMap.get(Servo.class, "hoodServoL");
        hoodServoR = hwMap.get(Servo.class, "hoodServoR");

        gateServo = hwMap.get(Servo.class, "gate");

        turretLimitSwitch = hwMap.get(RevTouchSensor.class, "turretLimitSwitch");

        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");

        RGBLight1 = hwMap.get(Servo.class, "rgb1");
        RGBLight2 = hwMap.get(Servo.class, "rgb2");

        //limelight = hwMap.get(Limelight3A.class, "limelight");

        // reverse one side to make sure all motors are in sync
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretRotationMotor.setTargetPosition(0);
        turretRotationMotor.setPower(0);
        turretRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretRotationMotor.setZeroPowerBehavior(BRAKE);

        intakeMotor.setPower(0);
        launcherR.setVelocity(0);
        launcherL.setVelocity(0);

        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServoL.setDirection(Servo.Direction.FORWARD);
        hoodServoR.setDirection(Servo.Direction.REVERSE);

        gateServo.setPosition(0.5);
        hoodServoL.setPosition(1);
        hoodServoR.setPosition(0);

        RGBLight1.setPosition(1);
        RGBLight2.setPosition(1);

        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(76.2, -190.5); // x: 3in y: -7.5in
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //limelight.setPollRateHz(100);
        //limelight.start();
        //limelight.pipelineSwitch(0);


    }
}
