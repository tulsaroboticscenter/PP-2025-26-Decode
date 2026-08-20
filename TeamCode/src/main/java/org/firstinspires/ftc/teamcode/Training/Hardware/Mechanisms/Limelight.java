package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    Limelight3A limelight = null;
    LLResult latestResult = null;
    LLStatus latestStatus = null;

    Pose3D botPose = null;

    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(11);
        limelight.pipelineSwitch(0);
    }

    public void start(){
        limelight.start();
    }

    public void setPipeLine(int index){
        limelight.pipelineSwitch(index);
    }


    public LLResult getLatestResult(){
        latestResult = limelight.getLatestResult();
        return latestResult;
    }

}
