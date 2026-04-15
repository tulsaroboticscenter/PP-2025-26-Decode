package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

import java.util.concurrent.TimeUnit;

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

    public Pose2D lastPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    public ElapsedTime poseUpdateRuntime = new ElapsedTime();

    public void initTeleOp(HardwareMap hwMap)
    {
        turret.init(hwMap, true);
        intake.init(hwMap);
        lights.init(hwMap, true);
        drivetrain.init(hwMap);
        pinpoint.init(hwMap, true);
        poseUpdateRuntime.startTime();
        //limelight.init(hwMap, true);
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

        antiVibratoryCorrection(lastPose, pinpoint.getPosition(), opmode);
    }
    public void updateInitTeleOp()
    {
        lights.update();
        pinpoint.update();
    }

    public void updatePedro()
    {
        turret.update();
        lights.update();
    }

    public void antiVibratoryCorrection(Pose2D lastPose, Pose2D currentPose, OpMode opmode)
    {
        if (poseUpdateRuntime.time(TimeUnit.MILLISECONDS) > 1000.0 / TOLERANCE_DETECTION_HZ)
        {
            if ((currentPose.getHeading(AngleUnit.DEGREES) - lastPose.getHeading(AngleUnit.DEGREES) * TOLERANCE_DETECTION_HZ) < ROTATION_TOLERANCE_DEG && !drivetrain.isInputtingOutsideDeadzone(opmode))
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
