package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// This opmode is to demonstrate a conditional statement without code block
// designations. the curly brackets are optional in which case only the following
// command line is included in the conditional statement.
@Disabled
@TeleOp(name="if test")
public class ConditionalTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.a)
            telemetry.addData("button a",gamepad1.a);
            telemetry.addLine("not part of condition"); // this statement is NOT part of the conditional
    }
}