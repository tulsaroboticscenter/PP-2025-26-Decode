package org.firstinspires.ftc.teamcode.Libraries;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private LinearOpMode opMode;
    private double leftPower;
    private double rightPower;
    private double strafePower;

    public MechOps(HWProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
    }


    /**
    public void launch() {
        //if (robot.launcher.getVelocity() >= robot.LAUNCHER_MIN_VELOCITY) {
            robot.leftFeeder.setPower(robot.FULL_SPEED);
        //}
    }
    public void stopLaunch() {
        robot.leftFeeder.setPower(robot.STOP_SPEED);
    }
     **/

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

    public void writePose (Pose2D Pose, String fileName) {

        File x = AppUtil.getInstance().getSettingsFile(fileName + "X");
        File y = AppUtil.getInstance().getSettingsFile(fileName + "Y");
        File heading = AppUtil.getInstance().getSettingsFile(fileName + "Heading");

        ReadWriteFile.writeFile(x, Double.toString(Pose.getX(DistanceUnit.MM)));
        ReadWriteFile.writeFile(y, Double.toString(Pose.getY(DistanceUnit.MM)));
        ReadWriteFile.writeFile(heading, Double.toString(Pose.getHeading(AngleUnit.DEGREES)));

    }

    public Pose2D readPose (String fromFileName) {

        File xFile = AppUtil.getInstance().getSettingsFile(fromFileName + "X");
        File yFile = AppUtil.getInstance().getSettingsFile(fromFileName + "Y");
        File headingFile = AppUtil.getInstance().getSettingsFile(fromFileName + "Heading");
        opMode.telemetry.addLine("readPose - File Found");

        String xData = ReadWriteFile.readFile(xFile).trim();
        String yData = ReadWriteFile.readFile(yFile).trim();
        String headingData = ReadWriteFile.readFile(headingFile).trim();
        opMode.telemetry.addLine("readPose - Data Found");


        double x = Double.parseDouble(xData);
        double y = Double.parseDouble(yData);
        double heading = Double.parseDouble(headingData);

        return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading);

    }  // end of method readPose()
}
