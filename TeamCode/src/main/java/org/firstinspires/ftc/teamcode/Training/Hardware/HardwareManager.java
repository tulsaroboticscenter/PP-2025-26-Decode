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
 //   public ShooterHood shooterHood = new ShooterHood();
    public HoodServo hoodServo = new HoodServo();
    public GateServo gateServo = new GateServo();

    public HardwareManager(HardwareMap hwMap)
    {

        this.hwMap = hwMap;
    }

    public void init(){

        driveTrain.init(hwMap);
//        shooterHood.init(hwMap);
        hoodServo.init(hwMap);
        gateServo.init(hwMap);
    }
}
