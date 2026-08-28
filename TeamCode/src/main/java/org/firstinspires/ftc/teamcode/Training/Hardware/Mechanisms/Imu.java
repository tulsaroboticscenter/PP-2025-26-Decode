package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Imu {
    IMU imu;

    public void init(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new com.qualcomm.robotcore.hardware.IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }
    public void resetYaw(){
        imu.resetYaw();
    }
    public double getRobotYawPitchRollAnglesRadians(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }

    public YawPitchRollAngles getRobotYawPitchRollAngles(){
        return imu.getRobotYawPitchRollAngles();

    }





}
