package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="if test")
public class ConditionalTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.a)
            telemetry.addData("button a",gamepad1.a);
            telemetry.addLine("not part of condition");
    }
}