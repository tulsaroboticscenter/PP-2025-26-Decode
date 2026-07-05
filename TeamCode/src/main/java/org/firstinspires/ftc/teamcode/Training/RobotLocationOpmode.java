package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    opmode to demonstrate the use of the class containing heading methods
 */
@TeleOp(name="Robot Location",group="Test")
public class RobotLocationOpmode extends OpMode {
    RobotLocation robotLocation = new RobotLocation(0);

    @Override
    public void init() {
        robotLocation.setAngle(0);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            robotLocation.turn(0.1);
        } else
            if (gamepad1.b){
                robotLocation.turn(-0.1);
            }
        telemetry.addData("Location",robotLocation);
        telemetry.addData("Heading",robotLocation.getHeading());

    }

}
