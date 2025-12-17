package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight
{
    private Limelight3A limelight = null;

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        limelight = hwMap.get(Limelight3A.class, "limelight");


    }

    public void setPipeline(int index)
    {
        limelight.pipelineSwitch(index);
    }

}
