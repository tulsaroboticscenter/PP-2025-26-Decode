package org.firstinspires.ftc.teamcode.opModes.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.Field;
import org.firstinspires.ftc.teamcode.Libraries.GamepadEffects;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.RGBLightController;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;
import org.firstinspires.ftc.teamcode.Libraries.TurretTargeting;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Jaxon Blue Debug", group = "Autonomous")
public class JaxonTestAuto extends OpMode {

    /**

    This PedroPathing Auto is used to test before updating pathing on other Pedro autos

    **/

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(60,8, Math.toRadians(90));
    private final Pose parkPose = new Pose(81,36, Math.toRadians(180));

    private PathChain forwards;

    HWProfile robot = new HWProfile();
    MechOps ops = new MechOps(robot, this);
    Targeting target = new Targeting(robot);
    TurretTargeting turret = new TurretTargeting(robot, target);

    Pose2D goalPosition = Field.toPedro2D(Field.blueGoal);

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                ops.setHoodPosition(robot.HOOD_HIGH_POSITION);
                ops.setLauncherVelocity(robot.LAUNCHER_HIGH_VELOCITY);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTime() > 1500) {
                    setPathState(2);
                }
                break;

            case 2:
                ops.openGate();
                robot.intakeMotor.setPower(1);
                if (pathTimer.getElapsedTime() > 4000) {
                    robot.turretRotationMotor.setTargetPosition(0);
                    ops.closeGate();
                    ops.setLauncherVelocity(0);
                    robot.intakeMotor.setPower(0);
                    setPathState(3);
                }
                break;

            case 3:
                follower.followPath(forwards, false);
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        forwards = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void init()
    {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        telemetry.addLine("Start init");
        telemetry.update();

        robot.initPedro(hardwareMap, false);

        telemetry.addLine("Hardware Done! ");
        telemetry.update();

        follower = Constants.createFollower(hardwareMap);
        telemetry.addLine("Follower Done! ");
        telemetry.update();

        buildPaths();
        telemetry.addLine("Paths Built! ");
        telemetry.update();

        follower.setStartingPose(startPose);
        follower.update();

        robot.turret.setTarget(follower.getPose(), goalPosition);

        gamepad1.runLedEffect(GamepadEffects.wakeBlue);
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop()
    {
    }

    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop()
    {
        follower.update();
        autonomousPathUpdate();
        robot.turret.update();

        telemetry.addData("State", pathState);
        telemetry.update();
    }
}

