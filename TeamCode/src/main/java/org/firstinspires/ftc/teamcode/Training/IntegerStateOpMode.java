package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
