package org.firstinspires.ftc.teamcode.Training.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.HoodServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.GateServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.PinPoint;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.Imu;


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
    public Imu imu = new Imu();



    public void init(HardwareMap hwMap) {


        driveTrain.init(hwMap);
        hoodServo.init(hwMap);
        gateServo.init(hwMap);
        pinPoint.init(hwMap);
        imu.init(hwMap);

    }
}
