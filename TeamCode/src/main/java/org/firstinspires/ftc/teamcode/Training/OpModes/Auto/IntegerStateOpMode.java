package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
    example of a state machine using multiple 'if' statements
    difficult to read, difficult to debug but can be done

 */
@Disabled
@Autonomous(name="Integer State",group="Test")
public class IntegerStateOpMode extends OpMode {
    int opModeState;

    @Override
    public void init() {
        opModeState = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("Cur State",opModeState);
        if  (opModeState==0) {
            telemetry.addLine("Press A to exit state");
             if (gamepad1.a){
                opModeState=1;
             } else
                if (opModeState==1) {
                    telemetry.addLine("Press B to exit state");
                    if (gamepad1.a) {
                        opModeState = 1;
                    }
                }
                else
                    if (opModeState == 2){
                        telemetry.addLine("Press X to exit state");
                        if (gamepad1.b){
                            opModeState=2;
                        }
                    }
                else
                telemetry.addLine("state finished");
        }
    }
}
