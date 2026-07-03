package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test")

public class First_Test extends OpMode {
    int loopCount;
    double currentX;
    double currentY;
    private ElapsedTime runtime = new ElapsedTime();
    public void init() {
        telemetry.addData("hello", "Eli");
         loopCount = 0;
         //runtime.reset();
    }

    @Override
    public void loop() {
currentX = gamepad1.left_stick_x;
currentY = -gamepad1.left_stick_y;
    if (gamepad1.dpad_up){
            loopCount = loopCount + 1;
        } else{
        if (gamepad1.dpad_down) {
            loopCount = loopCount - 1;
        }
        else{
            if (gamepad1.a) {
                loopCount = 0;
            }
        }
        telemetry.addData("count", loopCount);
        telemetry.addData("currentX",currentX);
        telemetry.addData("currentY",currentY);

    }
    }
}
