package org.firstinspires.ftc.teamcode.opModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ZeroHood", group="Robot")
public class ZeroHood extends OpMode
{

    private HardwareManager hw = new HardwareManager();

    @Override
    public void init() {
        hw.turret.init(hardwareMap, false);
    }

    @Override
    public void loop() {

    }
}
