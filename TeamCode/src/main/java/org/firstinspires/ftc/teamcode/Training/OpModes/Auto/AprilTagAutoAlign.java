package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

@Autonomous(name = "April Tag Auto Align")

public class AprilTagAutoAlign extends OpMode {

    double kP = 0.002;
    double kD = 0.0001;
    double error = 0.0;
    double lastError = 0.0;
    double goalX = 0; // offest?
    double angleTolerance = 0.4;
    double curTime = 0.0;
    double lastTime = 0.0;
    double forward;
    double strafe;
    double rotate;
    double curPositionRadians = 0;
    double[] stepSizes = {.01, 0.01, .001, .0001};
    int stepIndex = 2;
    int[] pipelineArray = {1, 2};
    int pipelineIndex = 0;
    LLResult llResult = null;
    private HardwareManager hwMgr = new HardwareManager(hardwareMap);

    public void init() {
        hwMgr.init_drivetrain(hardwareMap);
        hwMgr.limelight.setPipeLine(pipelineArray[pipelineIndex]);

        telemetry.addLine("Initialzed");

    }

    public void start() {
        hwMgr.limelight.start();

        resetRuntime();
        curTime = getRuntime();

    }

    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        YawPitchRollAngles orientation = hwMgr.imu.getRobotYawPitchRollAngles();
        hwMgr.limelight.updateRobotOrientation(orientation.getYaw());
        llResult = hwMgr.limelight.getLatestResult();


        if (gamepad1.left_trigger > .3) {
            if (llResult.isValid() && llResult != null) {
                error = goalX - llResult.getTx();
                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;
                    curTime = getRuntime();
                    double difTime = curTime - lastTime;
                    double dTerm = ((error - lastError) / difTime) * kD;
                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }

        }
        curPositionRadians = hwMgr.imu.getRobotYawPitchRollAnglesRadians();
        hwMgr.driveTrain.driveRobotField(forward, strafe, rotate, curPositionRadians);

        if (gamepad1.bWasReleased()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        if (llResult != null && llResult.isValid()) {
            telemetry.addData("AT ", llResult.getTx());
        } else {
            telemetry.addLine("Manual mode");
        }
        telemetry.addData("kP ", kP);
        telemetry.addData("kD ", kD);
        telemetry.addData("step ", stepSizes[stepIndex]);
        telemetry.addData("Error ", error);

    }


}
