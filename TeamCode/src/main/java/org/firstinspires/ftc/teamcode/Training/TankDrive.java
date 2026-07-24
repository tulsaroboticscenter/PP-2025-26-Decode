package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

/*
    test opmode to simulate tank drive but with mecanum wheels

 */
@TeleOp(name="Tank Drive",group="Test")
public class TankDrive extends OpMode {

    private HardwareManager hwMgr = new HardwareManager(hardwareMap);
    @Override
    public void init() {
        hwMgr.init(hardwareMap);
    }

    @Override
    public void loop() {
        hwMgr.driveTrain.driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
    }
}
