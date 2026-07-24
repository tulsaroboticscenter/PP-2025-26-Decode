package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "class test")
public class TestOpMode extends OpMode {
    Test testClass = new Test();

    public void init(){
        telemetry.addLine("init done");
    }

    @Override
    public void loop(){
        double xValue = gamepad1.left_stick_x;
        double yValue = gamepad1.left_stick_y;
        telemetry.addData("x=",xValue);
        telemetry.addData("y=",yValue);
        double amountAdded = testClass.sumAmount(xValue,yValue);
        telemetry.addData("sumAmount", amountAdded);
        double amountSubtracted = testClass.subAmount(xValue, yValue);
        telemetry.addData("subAmount",amountSubtracted);

    }
}
