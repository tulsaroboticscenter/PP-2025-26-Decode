package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;


@Autonomous(name = "Pinpoint Example 2")
public class PinPointExample2 extends OpMode {
    Double posX;
    Double posY;
    Double curHeading;
    double[] speedArray = {.1, .25, .40, .55};
    int[] distanceArray = {12, 24, 36, 48};
    int speedIndex = 0;
    int distanceIndex = 0;
    private HardwareManager hwMgr = new HardwareManager(hardwareMap);
    private PathState pathState;
    private ElapsedTime pathTimer;

    @Override
    public void init() {
        hwMgr.init_drivetrain(hardwareMap);

        getPinpointData();
        pathTimer = new ElapsedTime();
        pathState = PathState.DRIVE_START_TO_FIRST_POSITION;

    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press A to change distance");
        telemetry.addLine("Press B to change speed");
        telemetry.addData("Distance ", distanceArray[distanceIndex]);
        telemetry.addData("Speed ", speedArray[speedIndex]);

        if (gamepad1.aWasPressed()) {
            distanceIndex = distanceIndex++ % distanceArray.length;
        }

        if (gamepad1.bWasPressed()) {
            speedIndex = speedIndex++ % speedArray.length;
        }

    }

    @Override
    public void loop() {
        telemetry.addData("Speed ", speedArray[speedIndex]);
        telemetry.addData("Distance ", distanceArray[distanceIndex]);

        getPinpointData();

        statePathUpdate();

    }

    private void getPinpointData() {
        hwMgr.pinPoint.pinPoint.update();
        Pose2D pose2D = hwMgr.pinPoint.pinPoint.getPosition();
        posX = pose2D.getX(DistanceUnit.INCH);
        posY = pose2D.getY(DistanceUnit.INCH);
        curHeading = pose2D.getHeading(AngleUnit.DEGREES);

        telemetry.addData("posX ", posX);
        telemetry.addData("posY ", posY);
        telemetry.addData("Heading ", curHeading);
    }

    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_TO_FIRST_POSITION:
                if (posX > distanceArray[distanceIndex]) {
                    hwMgr.driveTrain.driveRobotMecanum(0, 0, 0); // stop
                    setPathState(PathState.PARK);
                } else {
                    hwMgr.driveTrain.driveRobotField(speedArray[speedIndex], 0, 0, Math.toRadians(0));
                }

                break;

            case PARK:
                stop();
                break;
            default:
                telemetry.addLine("No state defined");
        }
    }

    public void setPathState(PathState newPathState) {
        pathState = newPathState;

        pathTimer.reset();
    }

    public enum PathState {
        DRIVE_START_TO_FIRST_POSITION,
        ROTATE_RIGHT,
        DRIVE_TO_SECOND_POSITION,
        PARK
    }

}
