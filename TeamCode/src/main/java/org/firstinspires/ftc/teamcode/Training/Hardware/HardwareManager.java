package org.firstinspires.ftc.teamcode.Training.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.HoodServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.GateServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.PinPoint;


/*
    class to instantiate and initialize hardware mechanisms as we add them
 */
public class HardwareManager {
    public HardwareMap hwMap = null;
    public HardwareManager(HardwareMap hwMap) {

        this.hwMap = hwMap;
    }

    public DriveTrain driveTrain = new DriveTrain();
    public HoodServo hoodServo = new HoodServo();
    public GateServo gateServo = new GateServo();
    public PinPoint pinPoint = new PinPoint();





    public void init(HardwareMap hwMap) {


        driveTrain.init(hwMap);
        hoodServo.init(hwMap);
        gateServo.init(hwMap);
        pinPoint.init(hwMap);


    }
}
