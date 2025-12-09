package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.Field;
import org.firstinspires.ftc.teamcode.Libraries.GamepadEffects;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.RGBLightController;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;
import org.firstinspires.ftc.teamcode.Libraries.TurretTargeting;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous(name = "AutoBlue", group = "Robot", preselectTeleOp = "TeleBlue")
public class autoBlue extends LinearOpMode {

    private static final HWProfile robot = new HWProfile();
    private final MechOps ops = new MechOps(robot, this);
    private final Pose2D goalPosition = Field.blueGoal;

    enum States {
        SHOOT,
        PARK
    }

    States State = States.SHOOT;

    public void runOpMode() {
        robot.init(hardwareMap, false);
        while (robot.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            telemetry.addData("Status:", "Waiting on Pinpoint...");
            telemetry.update();
            robot.pinpoint.update();
        }
        robot.pinpoint.setPosition(Field.blueTouchingGoalFacingToward);
        robot.pinpoint.update();
        sleep(300);
        String data = String.format(Locale.US, "{%.1f, %.1f} %.1f degrees", (robot.pinpoint.getPosX() / 25.4), (robot.pinpoint.getPosY() / 25.4), Math.toDegrees(robot.pinpoint.getHeading()));
        telemetry.addData("Starting Position: ", data);
        telemetry.addData("Current state", State);
        telemetry.update();

        gamepad1.runLedEffect(GamepadEffects.wakeBlue);

        ElapsedTime preloadingToggleRuntime = new ElapsedTime();
        preloadingToggleRuntime.reset();
        boolean preloading = false;

        ops.setRGB(0.611);
        ops.setRGBMode(RGBLightController.LEDMode.PULSE_WAKE);
        while (opModeInInit())
        {
            ops.updateRGB();

            if (gamepad1.b && preloadingToggleRuntime.seconds() > 0.5)
            {
                if (preloading)
                    robot.intakeMotor.setPower(0);
                else
                    robot.intakeMotor.setPower(1);

                preloading = !preloading;
                preloadingToggleRuntime.reset();
            }
        }

        waitForStart();

        if (opModeIsActive()) {
            robot.turret.update();

            switch (State){
                case SHOOT:

                    ops.setLauncherVelocity(robot.LAUNCHER_LOW_VELOCITY);
                    ops.setHoodPosition(robot.HOOD_LOW_POSITION);
                    ops.setAllMotors(-0.3);
                    sleep(700);
                    ops.allStop();
                    sleep(200);

                    // Aim
                    robot.turret.setTarget(robot.pinpoint.getPosition(), goalPosition);
                    while (robot.launcherR.getVelocity() < robot.LAUNCHER_LOW_VELOCITY - 100) {
                        sleep(10);
                    }
                    robot.intakeMotor.setPower(1);
                    robot.gateServo.setPosition(0.8);
                    sleep(6000);
                    robot.intakeMotor.setPower(0);
                    ops.setLauncherVelocity(0);

                    //park
                    robot.leftFrontDrive.setPower(0.3);
                    robot.leftBackDrive.setPower(0.3);
                    robot.rightFrontDrive.setPower(-0.3);
                    robot.rightBackDrive.setPower(-0.3);
                    sleep(700);
                    ops.setAllMotors(-0.3);
                    sleep(1500);
                    ops.allStop();
                    sleep(2000);
                    State = States.PARK;
                    break;

                case PARK:
                    /*robot.leftFrontDrive.setPower(0.3);
                    robot.leftBackDrive.setPower(0.3);
                    robot.rightFrontDrive.setPower(-0.3);
                    robot.rightBackDrive.setPower(-0.3);
                    sleep(700);
                    ops.setAllMotors(-0.3);
                    sleep(2000);
                    ops.allStop();
                    sleep(2000);
                    break;*/
            }

            robot.pinpoint.update();

            telemetry.addData("Ending Position: ", robot.pinpoint.getPosX() + ", " + robot.pinpoint.getPosY());
            telemetry.addData("Current state", State);
            telemetry.update();
            ops.writePose(robot.pinpoint.getPosition(), "PositionFile");
        }
    }
}
