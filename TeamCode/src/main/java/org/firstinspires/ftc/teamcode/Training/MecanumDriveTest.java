package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.GateServo;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.HoodServo;

/*
    test opmode for mecanum wheels

 */
@TeleOp(name="Mecanum Drive",group="Test")
public class MecanumDriveTest extends OpMode {

    private HardwareManager hwMgr = new HardwareManager(hardwareMap);
    @Override
    public void init() {
        hwMgr.init(hardwareMap);
    }

    @Override
    public void loop() {
        hwMgr.driveTrain.driveRobotTank(-gamepad1.left_stick_y, gamepad1.left_stick_x);


    }
}
