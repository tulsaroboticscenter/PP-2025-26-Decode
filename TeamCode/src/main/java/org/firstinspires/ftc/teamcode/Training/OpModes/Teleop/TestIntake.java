package org.firstinspires.ftc.teamcode.Training.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

/*
    test opmode for mecanum wheels

 */
@TeleOp(name="intake test",group="Test")
public class TestIntake extends OpMode {

    double curPosRadians;
    boolean fieldCentric = false;
    private HardwareManager hwMgr = new HardwareManager(hardwareMap);

    @Override
    public void init() {

        hwMgr.init_drivetrain(hardwareMap);

        fieldCentric = false;

        telemetry.addLine("Press A to reset IMU");
        telemetry.addLine("Press X to toggle robot or field centric");
        telemetry.addData("Field centric =",fieldCentric);
    }

    @Override
    public void loop() {
        // reset imu if button A pressed
        if (gamepad1.a){
            hwMgr.imu.resetYaw();

        }

        // if button X, toggle field centric
        if (gamepad1.x){
            fieldCentric = !fieldCentric;
        }
        if (fieldCentric){
            // field centric
            curPosRadians = hwMgr.imu.getRobotYawPitchRollAnglesRadians();
            hwMgr.driveTrain.driveRobotField(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, curPosRadians);
        } else {
            // robot centric
            hwMgr.driveTrain.driveRobotMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
       if (gamepad1.y){
           hwMgr.intakeMotor.intake(1);
       }
       if (gamepad1.b){
           hwMgr.intakeMotor.intake(0);
       }
        telemetry.addData("Field centric =",fieldCentric);



    }
}
