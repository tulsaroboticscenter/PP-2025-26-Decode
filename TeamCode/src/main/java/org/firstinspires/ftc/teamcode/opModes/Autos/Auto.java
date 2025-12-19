package org.firstinspires.ftc.teamcode.opModes.Autos;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw = new HardwareManager();

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    private final Pose startPose = new Pose(23.7, 129.2, Math.toRadians(144));
    private final Pose scorePose = new Pose(59.9, 84.1, Math.toRadians(180));
    private final Pose intake1 = new Pose(16.3, 84.1, Math.toRadians(180));
    private final Pose prepIntake2 = new Pose(42.8, 59.7, Math.toRadians(180));
    private final Pose intake2 = new Pose(12.6, 59.5, Math.toRadians(180));
    private final Pose prepIntake3 = new Pose(42.8, 35.8, Math.toRadians(180));
    private final Pose intake3 = new Pose(12.6, 35.8, Math.toRadians(180));
    private final Pose park = new Pose(59.7, 103.5);

    private PathChain scorePreload, intakeLine1, scoreLine1, lineupIntake2, intakeLine2, scoreLine2, lineupIntake3, intakeLine3, scoreLine3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        intakeLine1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scoreLine1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        lineupIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepIntake2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeLine2 = follower.pathBuilder()
                .addPath(new BezierLine(prepIntake2, intake2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scoreLine2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        lineupIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prepIntake3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakeLine3 = follower.pathBuilder()
                .addPath(new BezierLine(prepIntake3, intake3))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scoreLine3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, park))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                hw.turret.setTarget(follower.getPose(), goalPosition);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shooterTimer.resetTimer();


                    if (shooterTimer.getElapsedTime() > 4000) {
                        follower.followPath(intakeLine1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLine1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(lineupIntake2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(intakeLine2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLine2, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(lineupIntake3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(intakeLine3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLine3, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    setPathState(-1);
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

        // Constantly Save the Position
        Field.lastKnownPosition = new Pose2D(DistanceUnit.MM, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.RADIANS, follower.getHeading());

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addLine("Position: " + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
        goalPosition = ((currentSide == Field.Side.RED) ? Field.redGoal : Field.blueGoal);

        // Save the selected alliance side, so TeleOp can read it and automatically load the goal position.
        Field.lastAllianceSide = currentSide;
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init()
    {
        pathTimer = new Timer();
        shooterTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.RED);

        shotsFired = false;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side currentSide = Field.Side.BLUE;
    private Field.StartingPosition currentStartingPosition = Field.StartingPosition.NEAR;
    @Override
    public void init_loop()
    {
        // THIS CURRENTLY DOES NOT DO ANYTHING. THIS AUTO WILL ONLY RUN THE CURRENTLY PROGRAMMED BRANCH
        if (gamepad1.squareWasPressed())
        {
            hw.lights.setLightColor(((hw.lights.getLightColor() == RGBLightController.RED) ? RGBLightController.BLUE : RGBLightController.RED));
            currentSide = ((currentSide == Field.Side.RED) ? Field.Side.BLUE : Field.Side.RED);
        }

        if (gamepad1.triangleWasPressed())
        {
            currentStartingPosition = ((currentStartingPosition == Field.StartingPosition.NEAR) ? Field.StartingPosition.FAR : Field.StartingPosition.NEAR);
        }

        telemetry.addLine("Side: " + ((currentSide == Field.Side.RED) ? "Red" : "Blue"));
        telemetry.addLine("Starting Position: " + ((currentStartingPosition == Field.StartingPosition.NEAR) ? "Near" : "Far"));
        telemetry.addLine("Press [Square] to switch sides.");
        telemetry.addLine("Press [Triangle] to switch starting positions.");
    }
}