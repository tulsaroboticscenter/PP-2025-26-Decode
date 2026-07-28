package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name="test gamepad program")
public class GamepadExamples extends OpMode {
   private boolean initDone;
    @Override
    public void init() {
        telemetry.addData("init",initDone);
    initDone=true;
    }

    @Override
    public void init_loop() {
        super.init_loop();
        telemetry.addData("Loop",initDone);
    }

    @Override
    public void loop() {
        telemetry.addData("init",initDone);
        telemetry.addData("X value",gamepad1.left_stick_x);
        telemetry.addData("Y value",gamepad1.left_stick_y);
    }
}
