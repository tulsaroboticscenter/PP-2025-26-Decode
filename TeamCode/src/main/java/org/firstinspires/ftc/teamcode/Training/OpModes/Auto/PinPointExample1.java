package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

@TeleOp(name = "Pinpoint Example 1")
public class PinPointExample1 extends OpMode {
  private HardwareManager hwMgr = new HardwareManager(hardwareMap);
  Double PosX;
  Double PosY;
  Double CurHeading;


    @Override
    public void init() {
        hwMgr.init(hardwareMap);
        hwMgr.pinPoint.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

     }

    @Override
    public void loop() {

        hwMgr.pinPoint.pinPoint.update();
        Pose2D pose2D = hwMgr.pinPoint.pinPoint.getPosition();
        PosX = pose2D.getX(DistanceUnit.INCH);
        PosY = pose2D.getY(DistanceUnit.INCH);
        CurHeading = pose2D.getHeading(AngleUnit.DEGREES);



    }


}
