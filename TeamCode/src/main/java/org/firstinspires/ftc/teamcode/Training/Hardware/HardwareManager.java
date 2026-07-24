package org.firstinspires.ftc.teamcode.Training.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.HoodServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.GateServo;

/*
    class to instantiate and initialize hardware mechanisms as we add them
 */
public class HardwareManager {
    public HardwareMap hwMap = null;
    public DriveTrain driveTrain = new DriveTrain();
    public HoodServo hoodServo = new HoodServo();
    public GateServo gateServo = new GateServo();

    public HardwareManager(HardwareMap hwMap) {

        this.hwMap = hwMap;
    }

<<<<<<< HEAD
    public void init(HardwareMap hwMap){
=======
    public void init(HardwareMap hwMap) {
>>>>>>> 37c8a629381ad7fc7ec1e233f04c1073247ec1e1

        driveTrain.init(hwMap);
        hoodServo.init(hwMap);
        gateServo.init(hwMap);
    }
}
