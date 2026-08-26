package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;


@Autonomous(name = "Pinpoint Example 1")
public class PinPointExample1 extends OpMode {
  private HardwareManager hwMgr = new HardwareManager(hardwareMap);
  Double PosX;
  Double PosY;
  Double CurHeading;

  public enum PathState {
      DRIVE_START_TO_FIRST_POSITION,
      ROTATE_RIGHT,
      DRIVE_TO_SECOND_POSITION,
      PARK
  }

    private PathState pathState;
  private ElapsedTime pathTimer;

    @Override
    public void init() {
        hwMgr.init_drivetrain(hardwareMap);
        hwMgr.pinPoint.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        pathTimer = new ElapsedTime();
        pathState = PathState.DRIVE_START_TO_FIRST_POSITION;

     }

    @Override
    public void start() {
        super.start();

    }

    @Override
    public void loop() {

        hwMgr.pinPoint.pinPoint.update();
        Pose2D pose2D = hwMgr.pinPoint.pinPoint.getPosition();
        PosX = pose2D.getX(DistanceUnit.INCH);
        PosY = pose2D.getY(DistanceUnit.INCH);
        CurHeading = pose2D.getHeading(AngleUnit.DEGREES);

        statePathUpdate();

    }

    private void statePathUpdate(){
        switch(pathState) {
            case DRIVE_START_TO_FIRST_POSITION:
                if (PosY > 24) {
                    hwMgr.driveTrain.driveRobotMecanum(0,0,0); // stop
                    setPathState(PathState.ROTATE_RIGHT);
                } else {
                    hwMgr.driveTrain.driveRobotField(.5,0,0,Math.toRadians(0));
                }

                break;
            case ROTATE_RIGHT:
                if (CurHeading < 180 ){
                    hwMgr.driveTrain.driveRobotMecanum(0,0,.5); // rotate right
                } else {
                    hwMgr.driveTrain.driveRobotMecanum(0,0,0); // stop
                    setPathState(PathState.DRIVE_TO_SECOND_POSITION);
                }

                break;
            case DRIVE_TO_SECOND_POSITION:
                hwMgr.driveTrain.driveRobotMecanum(.5,0,0);
                if (PosX > 12 || pathTimer.seconds() > 2) {
                    setPathState(PathState.PARK);
                 }
                break;
            case PARK:
                stop();
                break;
            default:
                telemetry.addLine("No state defined");
        }
    }

    public void setPathState(PathState newPathState){
        pathState = newPathState;

        pathTimer.reset();
    }

}
