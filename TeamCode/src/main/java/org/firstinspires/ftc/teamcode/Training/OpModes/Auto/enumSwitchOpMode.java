package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
    demonstration of the ue of an enum data type for cleaner state machine
    code. The enum data types are more flexible and user friendly than integers or
    strings
 */
@Disabled
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
            case WAIT_FOR_A:  // initial state waiting for driver to press A
                telemetry.addLine("Press A to exit state");
                if (gamepad1.a){
                    opModeState = OpModeState.WAIT_FOR_B;
                }
                break;
            case WAIT_FOR_B: // state after A pressed, waiting for press of B
                telemetry.addLine("Press B to exit state");
                if (gamepad1.b){
                    opModeState = OpModeState.WAIT_FOR_X;
                }
                break;
            case WAIT_FOR_X:  // third state waiting to press of X
                telemetry.addLine("Press x to end");
                if (gamepad1.x) {
                    opModeState = OpModeState.FINISHED;
                }
                break;
            default: // final state
                telemetry.addLine("state finished");
        }
    }
}
