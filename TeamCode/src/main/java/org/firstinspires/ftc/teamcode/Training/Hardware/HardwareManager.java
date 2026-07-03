package org.firstinspires.ftc.teamcode.Training.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.DriveTrain;

public class HardwareManager {
    public HardwareMap hwMap = null;
    public DriveTrain driveTrain = new DriveTrain();

    public HardwareManager(HardwareMap hwMap)
    {

        this.hwMap = hwMap;
    }

    public void init(){

        driveTrain.init(hwMap);
    }
}
