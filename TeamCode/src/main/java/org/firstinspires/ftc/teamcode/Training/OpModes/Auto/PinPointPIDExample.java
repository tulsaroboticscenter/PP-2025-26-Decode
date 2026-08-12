package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;


@TeleOp(name = "Pinpoint PID Example")
public class PinPointPIDExample extends OpMode {
  private HardwareManager hwMgr = new HardwareManager(hardwareMap);
  Double PosX;
  Double PosY;
  Double CurHeading;

  static final double kP = 0.001;
  static final double kI = 0.01;
  static final double kD = 0.001;

    double lastError = 0.0;
    ElapsedTime PIDTimer;


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
        hwMgr.init_auto(hardwareMap);
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
        PIDTimer.reset();

        statePathUpdate();

    }

    private void statePathUpdate(){
        switch(pathState) {
            case DRIVE_START_TO_FIRST_POSITION:
                if (PosY > 24) {
                    hwMgr.driveTrain.driveRobotMecanum(0,0,0); // stop
                    setPathState(PathState.ROTATE_RIGHT);
                } else {
                    double power = setDistancePower(24,PosY);
                    hwMgr.driveTrain.driveRobotField(power,0,0,Math.toRadians(90));
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
                    double power = setDistancePower(12,PosX);
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

    public double setDistancePower(double desiredDistance, double currentDistance){
        double error = desiredDistance - currentDistance;
        double derivative = (error - lastError) / pathTimer.seconds();
        double power = (kP * error) + kI + (kD * derivative);

        PIDTimer.reset();
        lastError = error;

        return power;

    }

    public double setRotatePower(double desiredAngle, double currentAngle){
        double error = desiredAngle  - currentAngle;
        double derivative = (error - lastError) / pathTimer.seconds();
        double power = (kP * error) + kI + (kD * derivative);

        PIDTimer.reset();
        lastError = error;

        return power;

    }
}
