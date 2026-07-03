package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="ENUM State",group="Test")
public class enumSwitchOpMode extends OpMode {
    enum OpModeState {
        WAIT_FOR_A,
        WAIT_FOR_B,
        WAIT_FOR_X,
        FINISHED
    }

    OpModeState opModeState = OpModeState.WAIT_FOR_A;

    @Override
    public void init() {

        opModeState = OpModeState.WAIT_FOR_A;
    }

    @Override
    public void loop() {
        telemetry.addData("Cur State",opModeState);
        switch (opModeState) {
            case WAIT_FOR_A:
                telemetry.addLine("Press A to exit state");
                if (gamepad1.a){
                    opModeState = OpModeState.WAIT_FOR_B;
                }
                break;
            case WAIT_FOR_B:
                telemetry.addLine("Press B to exit state");
                if (gamepad1.b){
                    opModeState = OpModeState.WAIT_FOR_X;
                }
                break;
            case WAIT_FOR_X:
                telemetry.addLine("Press x to end");
                if (gamepad1.x) {
                    opModeState = OpModeState.FINISHED;
                }
                break;
            default:
                telemetry.addLine("state finished");
        }
    }
}
