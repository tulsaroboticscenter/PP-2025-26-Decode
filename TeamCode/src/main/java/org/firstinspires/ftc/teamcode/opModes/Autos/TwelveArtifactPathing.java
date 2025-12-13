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

@Autonomous(name = "RAAAAHHHH", group = "Autonomous")
public class TwelveArtifactPathing extends OpMode {

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

    private final Pose startPose = new Pose(60,8, Math.toRadians(90));
    private final Pose parkPose = new Pose(60,36, Math.toRadians(180));

    private Path forwards;
    private Path backwards;

    HWProfile robot = new HWProfile();
    MechOps ops = new MechOps(robot, this);
    Targeting target = new Targeting(robot);
    TurretTargeting turret = new TurretTargeting(robot, target);

    Pose2D goalPosition = Field.toPedro2D(Field.blueGoal);

    ElapsedTime timer = new ElapsedTime();

    public void buildPaths() {
        forwards = new Path(new BezierLine(startPose, parkPose));
        forwards.setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading());
    }

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

        robot.turret.setTarget(follower.getPose(), goalPosition);
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

    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop()
    {
        follower.update();
        robot.turret.update();

        follower.followPath(forwards, false);
    }
}

