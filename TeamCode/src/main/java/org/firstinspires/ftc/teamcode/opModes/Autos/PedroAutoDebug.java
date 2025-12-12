package org.firstinspires.ftc.teamcode.opModes.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Thread.sleep;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Blue Debug", group = "Autonomous")
public class PedroAutoDebug extends OpMode {

    /**

    This PedroPathing Auto is used to test before updating pathing on other Pedro autos

    **/

    private enum AutoState {
        AIMING,
        WAIT_TO_FIRE,
        FIRING,
        MOVING,
        DONE
    }

    private AutoState currentState = AutoState.AIMING;

    private Path forwards;
    private Path backwards;

    HWProfile robot = new HWProfile();
    MechOps ops = new MechOps(robot, this);
    Targeting target = new Targeting(robot);
    TurretTargeting turret = new TurretTargeting(robot, target);

    Pose2D goalPosition = Field.toPedro2D(Field.blueGoal);

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init()
    {
        robot.initPedro(hardwareMap, false);
        ops.setRGB(0.611);
        ops.setRGBMode(RGBLightController.LEDMode.PULSE_WAKE);
        ops.updateRGB();
        gamepad1.runLedEffect(GamepadEffects.wakeBlue);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Field.toPedro(Field.redSmallZone));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop()
    {
        follower.update();
        ops.updateRGB();
    }

    @Override
    public void start()
    {

        forwards = new Path(new BezierLine(new Pose(60,8), new Pose(60,36)));
        forwards.setBrakingStrength(5);
        follower.setMaxPower(1);
        forwards.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));

    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop()
    {
        follower.update();
        robot.turret.update();

        switch (currentState)
        {
            case AIMING:
                robot.turret.setTarget(follower.getPose(), goalPosition);
                ops.setHoodPosition(robot.HOOD_HIGH_POSITION);
                ops.setLauncherVelocity(robot.LAUNCHER_HIGH_VELOCITY);
                timer.reset();
                currentState = AutoState.WAIT_TO_FIRE;
                break;

            case WAIT_TO_FIRE:
                if (timer.seconds() > 1.5)
                {
                    currentState = AutoState.FIRING;
                    timer.reset();
                }
                break;

            case FIRING:
                ops.openGate();
                robot.intakeMotor.setPower(1);
                if (timer.seconds() > 4)
                {
                    robot.turretRotationMotor.setTargetPosition(0);
                    ops.closeGate();
                    ops.setLauncherVelocity(0);
                    robot.intakeMotor.setPower(0);
                    currentState = AutoState.MOVING;
                    timer.reset();
                }
                break;

            case MOVING:

                follower.followPath(forwards, false);
                if (!follower.isBusy())
                {
                    currentState = AutoState.DONE;
                }
                break;
        }
        telemetry.addData("State", currentState.toString());
        telemetry.update();



    }
}

