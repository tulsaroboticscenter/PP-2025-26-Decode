package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.Field;
import org.firstinspires.ftc.teamcode.Libraries.GamepadEffects;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.RGBLightController;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;
import org.firstinspires.ftc.teamcode.Libraries.TurretTargeting;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Pedro Far Blue", group = "Autonomous")
public class PedroAutoFarBlue extends OpMode {

    private static final HWProfile robot = new HWProfile();
    private final MechOps ops = new MechOps(robot, this);
    private final Targeting target = new Targeting(robot);
    private final TurretTargeting turret = new TurretTargeting(robot, target);
    private final Field markers = new Field();
    private final GamepadEffects gamepadEffects = new GamepadEffects();

    private final Pose2D goalPosition = markers.blueGoal;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(60, 12, Math.toRadians(90));
    private final Pose endPose = new Pose(60.000, 36.0, Math.toRadians(180));

    private Path moveOut;

    private ElapsedTime preloadingToggleRuntime = new ElapsedTime();
    private boolean preloading = false;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        moveOut = new Path(new BezierLine(startPose, endPose));
        moveOut.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                ops.openGate();
                ops.setLauncherVelocity(robot.LAUNCHER_HIGH_VELOCITY);
                ops.setHoodPosition(robot.HOOD_HIGH_POSITION);
                robot.turretRotationMotor.setTargetPosition(turret.HeadingToTurretTicks(target.getDegreesToTarget(ops.poseToPose2D(follower.getPose()), markers.blueGoal, false), AngleUnit.DEGREES));
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTime() > 5000)
                    setPathState(2);
                break;

            case 2:
                robot.intakeMotor.setPower(1);
                setPathState(3);
                break;

            case 3:
                if (pathTimer.getElapsedTime() > 3000) {
                    robot.intakeMotor.setPower(0);
                    ops.setLauncherVelocity(0);
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTime() > 1000) {
                    robot.turretRotationMotor.setTargetPosition(0);
                    ops.writePosePedro(ops.poseToPose2D(follower.getPose()), "PoseFile");
                    setPathState(4);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        //robot.turretRotationMotor.setTargetPosition(turret.HeadingToTurretTicks(target.getDegreesToTarget(ops.poseToPose2D(follower.getPose()), goalPosition, false), AngleUnit.DEGREES));


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init()
    {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.initPedro(hardwareMap, false);

        ops.setRGB(0.611);
        ops.setRGBMode(RGBLightController.LEDMode.PULSE_WAKE);
        ops.updateRGB();
        gamepad1.runLedEffect(gamepadEffects.wakeBlue);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        preloadingToggleRuntime.reset();

        robot.turretRotationMotor.setTargetPosition(0);
        robot.turretRotationMotor.setPower(1);
        robot.turretRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop()
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

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

