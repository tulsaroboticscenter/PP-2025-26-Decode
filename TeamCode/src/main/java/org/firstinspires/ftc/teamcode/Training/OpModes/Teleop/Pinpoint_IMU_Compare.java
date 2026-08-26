package org.firstinspires.ftc.teamcode.Training.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

@TeleOp(name="Pinpoint IMU Compare", group =  "test")
public class Pinpoint_IMU_Compare extends OpMode {
    private HardwareManager hwMgr = new HardwareManager(hardwareMap);
    double curIMURadians;
    Double PosX;
    Double PosY;
    Double curPPHeading;
    double curIMUHeading;

    public void init(){
        hwMgr.init_drivetrain(hardwareMap);
    }
    public void loop(){

        hwMgr.pinPoint.pinPoint.update();

        Pose2D pose2D = hwMgr.pinPoint.pinPoint.getPosition();
        PosX = pose2D.getX(DistanceUnit.INCH);
        PosY = pose2D.getY(DistanceUnit.INCH);
        curPPHeading = pose2D.getHeading(AngleUnit.DEGREES);

        curIMURadians = hwMgr.imu.getRobotYawPitchRollAnglesRadians();
        curIMUHeading = Math.toDegrees(curIMURadians);

        telemetry.addData("PosX ",PosX);
        telemetry.addData("PosY ",PosY);
        telemetry.addData("IMU degrees ",curIMUHeading);
        telemetry.addData("PP degrees ",curPPHeading);

    }
}
