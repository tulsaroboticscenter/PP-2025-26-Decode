package org.firstinspires.ftc.teamcode.Libraries;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import java.io.File;


public class MechOps {

    HWProfile robot = new HWProfile();
    private double leftPower;
    private double rightPower;
    private double strafePower;

    public void launch() {
        //if (robot.launcher.getVelocity() >= robot.LAUNCHER_MIN_VELOCITY) {
            robot.leftFeeder.setPower(robot.FULL_SPEED);
        //}
    }
    public void stopLaunch() {
        robot.leftFeeder.setPower(robot.STOP_SPEED);
    }

    public void allStop() {

        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);

    }

    public void writePose (Pose2D Pose, String fileName) {

        File PoseFile = AppUtil.getInstance().getSettingsFile(fileName);
        String poseData = Pose.getX(DistanceUnit.MM) + "," + Pose.getY(DistanceUnit.MM) + "," + Pose.getHeading(AngleUnit.DEGREES);
        ReadWriteFile.writeFile(PoseFile, poseData);

    }

    public Pose2D readPose (String fromFileName) {

        File poseFile = AppUtil.getInstance().getSettingsFile(fromFileName);
        String data = ReadWriteFile.readFile(poseFile).trim();
        String[] parts = data.split(",");
        if (parts.length == 3) {
            try {
                double x = Double.parseDouble(parts[0]);
                double y = Double.parseDouble(parts[1]);
                double heading = Math.toRadians(Double.parseDouble(parts[2])); // Convert back to radians
                return new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading);
            } catch (NumberFormatException e) {
                telemetry.addData("Error parsing Pose2D data from file: ", e.getMessage());
                return new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Default pose on error
            }
        } else {
            telemetry.addLine("Incorrect data format in Pose2D file.");
            return new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Default pose on error
        }
    }  // end of method readFromFile()
}
