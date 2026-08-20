package org.firstinspires.ftc.teamcode.Training.OpModes.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

/*
    test opmode for limelight april tag recognition

 */
@TeleOp(name="April Tag Test",group="Test")
public class LLAprilTag extends OpMode {

    double curPosRadians;
    boolean fieldCentric = false;

    int[] pipelineArray = {0,1};
    int pipelineIndex = 0;

    LLResult llResult = null;

    private HardwareManager hwMgr = new HardwareManager(hardwareMap);

    @Override
    public void init() {

        hwMgr.init_teleop(hardwareMap);

        fieldCentric = false;

        telemetry.addLine("Press A to reset IMU");
        telemetry.addLine("Press X to toggle robot or field centric");
        telemetry.addData("Field centric =",fieldCentric);
    }

    public void start(){
        hwMgr.limelight.start();
    }
    @Override
    public void loop() {
        // reset imu if button A pressed
        if (gamepad1.a) {
            hwMgr.imu.resetYaw();

        }

        // if button X, toggle field centric
        if (gamepad1.x) {
            fieldCentric = !fieldCentric;
        }
        if (fieldCentric) {
            // field centric
            curPosRadians = hwMgr.imu.getRobotYawPitchRollAnglesRadians();
            hwMgr.driveTrain.driveRobotField(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, curPosRadians);
        } else {
            // robot centric
            hwMgr.driveTrain.driveRobotMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        telemetry.addData("Field centric =", fieldCentric);

        if (gamepad1.dpad_up) {
            pipelineIndex++;
            if (pipelineIndex > 1) {
                pipelineIndex = 1;
            }
        } else if (gamepad1.dpad_down) {
            pipelineIndex--;
            if (pipelineIndex < 0) {
                pipelineIndex = 0;
            }
        }
        hwMgr.limelight.setPipeLine(pipelineArray[pipelineIndex]);

        if (llResult.isValid() && llResult != null) {
            Pose3D pose3D = llResult.getBotpose();
            telemetry.addData("botpose ", pose3D);
        } else {
            telemetry.addLine("target not found");
        }
    } // loop end


    }

