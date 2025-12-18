package org.firstinspires.ftc.teamcode.Libraries;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.ColorSpace;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import java.io.File;


public class MechOps {

    private HWProfile robot;
    private LinearOpMode linearOpMode = null;
    private OpMode opMode = null;
    private double leftPower;
    private double rightPower;
    private double strafePower;

    public MechOps(HWProfile myRobot, LinearOpMode myLinearOpMode) {
        robot = myRobot;
        linearOpMode = myLinearOpMode;
    }

    public MechOps(HWProfile myRobot, OpMode myOpMode)
    {
        robot = myRobot;
        opMode = myOpMode;
    }

    public Pose2D poseToPose2D(Pose pose)
    {
        return new Pose2D(DistanceUnit.INCH, pose.getX() - 72, pose.getY() - 72, AngleUnit.RADIANS, pose.getHeading());
    }

    public void setHoodPosition(double position)
    {

        if (position > 0.9)
        {
            robot.hoodServoL.setPosition(0.1);
            robot.hoodServoR.setPosition(0.9);
        }
        else
        {
            robot.hoodServoL.setPosition(1 - position);
            robot.hoodServoR.setPosition(position);
        }
    }

    public void setRGB(double color)
    {
        robot.light1.setColor(color);
        robot.light2.setColor(color);
    }

    public void openGate()
    {
        robot.gateServo.setPosition(0.8);
    }
    public void closeGate()
    {
        robot.gateServo.setPosition(0.5);
    }


    public void setRGBMode(RGBLightController.LEDMode mode)
    {
        robot.light1.setMode(mode);
        robot.light2.setMode(mode);
    }

    public void updateRGB()
    {
        robot.light1.update();
        robot.light2.update();
    }

    public void setLauncherVelocity(double velocity)
    {
        robot.launcherL.setVelocity(velocity);
        robot.launcherR.setVelocity(velocity);
    }

    public double getLauncherRPM() {
        return (((robot.launcherL.getVelocity() + robot.launcherR.getVelocity()) / 2) / 28) * 60;
    }

    public void setAllMotors(double power) {
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    public void allStop() {

        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);

    }

    public void writePose (Pose2D Pose, String fileName)
    {

        File x = AppUtil.getInstance().getSettingsFile(fileName + "X");
        File y = AppUtil.getInstance().getSettingsFile(fileName + "Y");
        File heading = AppUtil.getInstance().getSettingsFile(fileName + "Heading");

        ReadWriteFile.writeFile(x, Double.toString(Pose.getX(DistanceUnit.MM)));
        ReadWriteFile.writeFile(y, Double.toString(Pose.getY(DistanceUnit.MM)));
        ReadWriteFile.writeFile(heading, Double.toString(Pose.getHeading(AngleUnit.DEGREES)));

    }

    public int turretDegreesToTicks (double angle) {
        int ticks = (int) (angle * 1.913);
        return ticks;
    }

    public void writePosePedro (Pose2D Pose, String fileName)
    {

        File x = AppUtil.getInstance().getSettingsFile(fileName + "X");
        File y = AppUtil.getInstance().getSettingsFile(fileName + "Y");
        File heading = AppUtil.getInstance().getSettingsFile(fileName + "Heading");

        ReadWriteFile.writeFile(x, Double.toString((Pose.getX(DistanceUnit.MM) - (25.4 * 72))));
        ReadWriteFile.writeFile(y, Double.toString(Pose.getY(DistanceUnit.MM) - (25.4 + 72)));
        ReadWriteFile.writeFile(heading, Double.toString(Pose.getHeading(AngleUnit.DEGREES)));

    }

    public Pose2D readPose (String fromFileName)
    {

        File xFile = AppUtil.getInstance().getSettingsFile(fromFileName + "X");
        File yFile = AppUtil.getInstance().getSettingsFile(fromFileName + "Y");
        File headingFile = AppUtil.getInstance().getSettingsFile(fromFileName + "Heading");
        //opMode.telemetry.addLine("readPose - File Found");

        String xData = ReadWriteFile.readFile(xFile).trim();
        String yData = ReadWriteFile.readFile(yFile).trim();
        String headingData = ReadWriteFile.readFile(headingFile).trim();
        //opMode.telemetry.addLine("readPose - Data Found");


        double x = Double.parseDouble(xData);
        double y = Double.parseDouble(yData);
        double heading = Double.parseDouble(headingData);

        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading);

    }  // end of method readPose()
}
