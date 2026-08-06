package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

public class PinPoint {
    public GoBildaPinpointDriver pinPoint = null;


    public void init(HardwareMap hwMap){
        pinPoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinPoint.setOffsets(76.2,-190.5);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinPoint.resetPosAndIMU();

        pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

}
