package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight
{
    private Limelight3A limelight = null;

    LLResult latestResult = null;
    LLStatus latestStatus = null;

    boolean seesTarget = false;

    Pose3D botPose = null;

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (50 times per second)
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    public void update(double turretYawDegrees)
    {
        limelight.updateRobotOrientation(turretYawDegrees);
        latestResult = limelight.getLatestResult();
        latestStatus = limelight.getStatus();

        if (latestResult.isValid() && latestResult != null)
        {
            seesTarget = true;
            botPose = latestResult.getBotpose_MT2();
        }
        else
        {
            seesTarget = false;
        }
    }

    public LLResult getLatestResult()
    {
        return latestResult;
    }

    public LLStatus getLatestStatus() {
        return latestStatus;
    }

    public void setPipeline(int index)
    {
        limelight.pipelineSwitch(index);
    }

    public boolean seesTarget()
    {
        return seesTarget;
    }

    public Pose2D getRobotPosition(double turretYawDegrees)
    {
        return new Pose2D(DistanceUnit.METER, botPose.getPosition().y, -botPose.getPosition().x, AngleUnit.DEGREES, botPose.getOrientation().getYaw(AngleUnit.DEGREES) - turretYawDegrees);
    }
}