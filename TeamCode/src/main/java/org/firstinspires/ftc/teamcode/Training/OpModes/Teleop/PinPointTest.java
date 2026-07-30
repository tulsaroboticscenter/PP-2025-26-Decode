package org.firstinspires.ftc.teamcode.Training.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

@TeleOp(name = "Pinpoint test")
public class PinPointTest extends OpMode {
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
     }

    @Override
    public void loop() {
        telemetry.addLine("Push robot to view tracking");
        telemetry.addLine("Press A to reset position");

        if (gamepad1.a){
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }

        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X in inches = ",pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y in inches = ",pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading in degrees = ",pose2D.getHeading(AngleUnit.DEGREES));

    }

    public void configurePinpoint(){
        pinpoint.setOffsets(76.2,-190.5);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

    }
}
