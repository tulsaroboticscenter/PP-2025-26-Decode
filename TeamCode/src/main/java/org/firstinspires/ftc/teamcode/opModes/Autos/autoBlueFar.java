package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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


@Autonomous(name = "AutoBlueFar", group = "Robot", preselectTeleOp = "TeleBlueFar")
public class autoBlueFar extends LinearOpMode {

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
        robot.pinpoint.setPosition(Field.redSmallZone);
        robot.pinpoint.update();
        sleep(300);
        String data = String.format(Locale.US, "{%.1f, %.1f} %.1f degrees", (robot.pinpoint.getPosX() / 25.4), (robot.pinpoint.getPosY() / 25.4), Math.toDegrees(robot.pinpoint.getHeading()));
        telemetry.addData("Starting Position: ", data);
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

                    robot.pinpoint.update();

                    // Aim
                    ops.setLauncherVelocity(robot.LAUNCHER_HIGH_VELOCITY);
                    ops.setHoodPosition(robot.HOOD_HIGH_POSITION);
                    robot.turret.setTarget(robot.pinpoint.getPosition(), goalPosition);
                    sleep(2000);

                    while (robot.launcherR.getVelocity() < robot.LAUNCHER_HIGH_VELOCITY - 100) {
                        sleep(10);
                    }
                    ops.openGate();
                    robot.intakeMotor.setPower(1);
                    sleep(6000);
                    ops.closeGate();
                    robot.intakeMotor.setPower(0);
                    ops.setLauncherVelocity(0);
                    sleep(200);
                    State = States.PARK;

                case PARK:
                    ops.setAllMotors(0.3);
                    sleep(1500);
                    ops.allStop();
                    sleep(2000);
                    break;
            }

            robot.pinpoint.update();

            telemetry.addData("Ending Position: ", robot.pinpoint.getPosX() + ", " + robot.pinpoint.getPosY());
            telemetry.update();
            Pose2D savingPose = new Pose2D(DistanceUnit.INCH, robot.pinpoint.getPosX(), robot.pinpoint.getPosY(), AngleUnit.DEGREES, (Math.toDegrees(robot.pinpoint.getHeading()) + 90));
            ops.writePose(savingPose, "PositionFile");
        }
    }
}
