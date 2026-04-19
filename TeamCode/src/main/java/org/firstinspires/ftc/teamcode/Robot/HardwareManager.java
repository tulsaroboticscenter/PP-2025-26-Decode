package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.concurrent.TimeUnit;

@Configurable
public class HardwareManager {


    public HardwareMap hwMap = null;
    public HardwareManager(HardwareMap hwMap)
    {
        this.hwMap = hwMap;
    }

    public Turret turret = new Turret();
    public Intake intake = new Intake();
    public Drivetrain drivetrain = new Drivetrain();
    public Lights lights = new Lights();
    public Pinpoint pinpoint = new Pinpoint();
    public Limelight limelight = new Limelight();

    public final double ROTATION_TOLERANCE_DEG = 5; // per second
    public final double TOLERANCE_DETECTION_HZ = 240.0;

    public final double TRANSALATIONAL_TOLERANCE_MM_PER_SEC = 5.0;

    public Pose2D lastPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    public ElapsedTime poseUpdateRuntime = new ElapsedTime();

    //public Follower follower;

    public void initTeleOp(HardwareMap hwMap)
    {
        turret.init(hwMap, true);
        intake.init(hwMap);
        lights.init(hwMap, true);
        drivetrain.init(hwMap);
        pinpoint.init(hwMap, true);
        poseUpdateRuntime.startTime();
        //limelight.init(hwMap, true);
        //follower = Constants.createFollower(hwMap);
    }

    public void initPedro(HardwareMap hwMap)
    {
        turret.init(hwMap, false);
        lights.init(hwMap, false);
        intake.init(hwMap);
    }

    public void updateTeleOp(OpMode opmode)
    {
        turret.update();
        lights.update();
        pinpoint.update();
        intake.update();
        //follower.update();

        //antiVibratoryCorrection(lastPose, pinpoint.getPosition(), opmode);
    }
    public void updateInitTeleOp()
    {
        lights.update();
        pinpoint.update();
        //follower.update();
    }

//    public void setDrivePowers(Gamepad gamepad, boolean fieldCentric, Pose2D goalPosition, Field.Side side) {
//
//        double x = -gamepad.left_stick_x;
//        double y = gamepad.left_stick_y;
//        double heading = -gamepad.right_stick_x;
//
//        if (follower.isBusy()) {
//            follower.breakFollowing();
//        }
//
//        Pose robotPose = follower.getPose();
//        Vector headingVector = new Vector();
//        Vector driveVector = new Vector();
//        double degreesToTarget = Turret.getHeading(follower.getPose(), goalPosition);
//
//        if (drivetrain.isTargeting) {
//            double robotHeading = MathFunctions.normalizeAngle(robotPose.getHeading());
//            double direction = MathFunctions.getTurnDirection(robotHeading, degreesToTarget);
//
//            degreesToTarget = MathFunctions.normalizeAngle(degreesToTarget);
//
//            headingVector = follower.getVectorCalculator().getHeadingVector(
//                    MathFunctions.getSmallestAngleDifference(robotHeading, degreesToTarget) * direction,
//                    robotPose,
//                    degreesToTarget
//            );
//        } else {
//            headingVector.setComponents(heading, robotPose.getHeading());
//        }
//
//        driveVector.setOrthogonalComponents(x, y);
//        driveVector.setMagnitude(Range.clip(driveVector.getMagnitude(), 0, 1));
//
//        if (fieldCentric) {
//            if (side == Field.Side.BLUE) {
//            driveVector.rotateVector(Math.PI / 2);
//            } else {
//                driveVector.rotateVector(-(Math.PI / 2));
//            }
//        } else {
//            driveVector.rotateVector(robotPose.getHeading());
//        }
//
//        follower.getDrivetrain().runDrive(
//                new Vector(),
//                headingVector,
//                driveVector,
//                robotPose.getHeading(),
//                follower.getVelocity()
//        );
//    }

    public void updatePedro()
    {
        turret.update();
        lights.update();
    }

    public void antiVibratoryCorrection(Pose2D lastPose, Pose2D currentPose, OpMode opmode)
    {
        if (poseUpdateRuntime.time(TimeUnit.MILLISECONDS) > 1000.0 / TOLERANCE_DETECTION_HZ)
        {
            if ((currentPose.getHeading(AngleUnit.DEGREES) - lastPose.getHeading(AngleUnit.DEGREES) * TOLERANCE_DETECTION_HZ) < ROTATION_TOLERANCE_DEG
                    && !drivetrain.isInputtingOutsideDeadzone(opmode)
                    && pinpoint.getVelocityR() < TRANSALATIONAL_TOLERANCE_MM_PER_SEC)
            {
                pinpoint.setPosition(lastPose);
            }
            else
            {
                lastPose = currentPose;
            }
            poseUpdateRuntime.reset();
        }
    }
}
