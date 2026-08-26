package org.firstinspires.ftc.teamcode.Training.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;

/*
    test drive opmode for mecanum wheels using pinpoint instead of IMU

 */
@Disabled
@TeleOp(name="Pinpoint Drive Shell",group="Test")
public class DriveShellPinpoint extends OpMode {

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
            hwMgr.pinPoint.pinPoint.resetPosAndIMU();
        }

        // if button X, toggle field centric
        if (gamepad1.x){
            fieldCentric = !fieldCentric;
        }
        if (fieldCentric){
            // field centric
            hwMgr.pinPoint.pinPoint.update();
            Pose2D pose2D = hwMgr.pinPoint.pinPoint.getPosition();
            curPosRadians = pose2D.getHeading(AngleUnit.RADIANS);
            hwMgr.driveTrain.driveRobotField(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, curPosRadians);
        } else {
            // robot centric
            hwMgr.driveTrain.driveRobotMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        telemetry.addData("Field centric =",fieldCentric);



    }
}
