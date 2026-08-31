package org.firstinspires.ftc.teamcode.Training.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;


@Autonomous(name = "Pinpoint PID Example")
public class PinPointPIDExample extends OpMode {
  private HardwareManager hwMgr = new HardwareManager(hardwareMap);
  Double PosX;
  Double PosY;
  Double CurHeading;
    double power = 0.0;
    double kP = 0.002;
   double kD = 0.0001;
    double error = 0.0;
    double lastError = 0.0;
    double pTerm;
    double dTerm;
    double goalX = 0; // offest?
    double distanceTolerance = 0.25;
    double curTime = 0.0;
    double lastTime = 0.0;
    double difTime;
    double forward;
    double strafe;
    double rotate;
    double curPositionRadians = 0;
    double[] speedArray = {.1,.25,.40,.55};
    int[] distanceArray = {12,24,36,48};
    int speedIndex = 0;
    int distanceIndex = 0;
    double[] stepSizes = {.01, 0.01, .001, .0001};
    int stepIndex = 2;

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
    public void init_loop() {
        telemetry.addLine("Press A to change distance");
        telemetry.addLine("Press B to change speed");
        telemetry.addLine("Press X to change step size");
        telemetry.addLine("Use dpad left/right to change kP");
        telemetry.addLine("Use dpad up/down to change kD");
         telemetry.addData("Distance ", distanceArray[distanceIndex]);
         telemetry.addData("Speed ", speedArray[speedIndex]);
         telemetry.addData("Step ", stepSizes[stepIndex]);
         telemetry.addData("kP ", kP);
         telemetry.addData("kD ", kD);

        if (gamepad1.aWasPressed()) {
            distanceIndex = distanceIndex++ % distanceArray.length;
        }

        if (gamepad1.bWasPressed()) {
            speedIndex = speedIndex++ % speedArray.length;
        }

         if (gamepad1.xWasPressed()) {
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

    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();

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
                power = setDistancePower(distanceArray[distanceIndex],PosX);
                hwMgr.driveTrain.driveRobotField(power,0,0,Math.toRadians(0));
                if (power == 0){
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
        error = desiredDistance - currentDistance;
        if (Math.abs(error) < distanceTolerance){
            power = 0;
        } else {
            pTerm = error * kP;
            curTime = getRuntime();
            difTime = curTime - lastTime;
            dTerm = ((error - lastError) / difTime) * kD;
            power = Range.clip(pTerm + dTerm, -1.0, 1.0);
            lastError = error;
            lastTime = curTime;
        }

        return power;

    }


}
