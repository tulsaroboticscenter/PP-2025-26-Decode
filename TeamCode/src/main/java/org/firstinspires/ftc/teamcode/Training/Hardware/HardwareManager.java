package org.firstinspires.ftc.teamcode.Training.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.HoodServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.GateServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.Limelight;
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
    public Intake intakeMotor = new Intake();
    public Limelight limelight = new Limelight();


    /* auto initialize - uses pedro pathing to initialize drivetrain so
    we do not intialize the drivetrain in auto
    */
    public void init_pedro(HardwareMap hwMap) {

        hoodServo.init(hwMap);
        gateServo.init(hwMap);
 //       pinPoint.init(hwMap);
 //       imu.init(hwMap);
        intakeMotor.init(hwMap);
        limelight.init(hwMap);

    }
    public void init_drivetrain(HardwareMap hwMap) {

        driveTrain.init(hwMap);
        hoodServo.init(hwMap);
        gateServo.init(hwMap);
        pinPoint.init(hwMap);
        imu.init(hwMap);
        intakeMotor.init(hwMap);
        limelight.init(hwMap);

    }
}
