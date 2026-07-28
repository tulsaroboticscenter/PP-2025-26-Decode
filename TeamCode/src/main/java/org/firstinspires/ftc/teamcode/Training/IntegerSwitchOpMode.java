package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    demonstration opmode for state machine using the switch statement
    a little cleaner but still difficult to know what each state represents
 */
@Disabled
@Autonomous(name="Integer Switch",group="Test")
public class IntegerSwitchOpMode extends OpMode {
    int opModeState;

    @Override
    public void init() {
        opModeState = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("Cur State",opModeState);
        switch (opModeState) {
            case 0:
                telemetry.addLine("Press A to exit state");
                if (gamepad1.a){
                    opModeState=1;
                }
                break;
            case 1: // hard to know what this state is supposed to do
                telemetry.addLine("Press B to exit state");
                if (gamepad1.b){
                    opModeState=2;
                }
                break;
            case 2:
                telemetry.addLine("Press x to end");
                if (gamepad1.x) {
                    opModeState = 3;
                }
                break;
            default:
                telemetry.addLine("state finished");
        }
    }
}
