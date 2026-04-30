package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Near", group = "Autonomous", preselectTeleOp = "tele")
public class RedNear extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw = new HardwareManager(hardwareMap);

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    private final Pose startPose = new Pose(117.18446601941747, 133.11650485436894, Math.toRadians(-53.322));
    private final Pose scorePose = new Pose(85, 84.1, Math.toRadians(0));
    private final Pose intake1 = new Pose(120.7, 84.1, Math.toRadians(0));
    private final Pose prepIntake2 = new Pose(95.2, 65, Math.toRadians(0));
    private final Pose intake2 = new Pose(125.4, 59.5, Math.toRadians(0));
    private final Pose prepIntake3 = new Pose(95.2, 40, Math.toRadians(0));
    private final Pose intake3 = new Pose(125.4, 35.8, Math.toRadians(0));
    private final Pose park = new Pose(110.942, 90, Math.toRadians(-90));


    private PathChain scorePreload, parkPath, intakeLine1, scoreLine1, lineupIntake2, intakeLine2, scoreLine2, lineupIntake3, intakeLine3, scoreLine3;


    public void buildPaths() {
        switch (numOfSpikes)
        {
            case 3:
                lineupIntake3 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, prepIntake3))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                intakeLine3 = follower.pathBuilder()
                        .addPath(new BezierLine(prepIntake3, intake3))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                scoreLine3 = follower.pathBuilder()
                        .addPath(new BezierLine(intake3, scorePose))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

            case 2:
                lineupIntake2 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, prepIntake2))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                intakeLine2 = follower.pathBuilder()
                        .addPath(new BezierLine(prepIntake2, intake2))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                scoreLine2 = follower.pathBuilder()
                        .addPath(new BezierLine(intake2, scorePose))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

            case 1:
                intakeLine1 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, intake1))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                scoreLine1 = follower.pathBuilder()
                        .addPath(new BezierLine(intake1, scorePose))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

            case 0:
                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, scorePose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                        .build();

            default:
        }
    }

    public void Park()
    {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), park))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        follower.followPath(parkPath);
    }

    public void autonomousPathRedUpdate() {
        switch (pathState) {
            // Back up to shoot
            case 0:
                hw.turret.spinUpFlywheel();
                hw.intake.intake();
                telemetry.addLine("spinning up flywheel-completed");

                follower.followPath(scorePreload, true);
                shooterTimer.resetTimer();
                setPathState(1);
                break;

                // Intake
            case 1:
                if (!follower.isBusy())
                {
                    hw.intake.intake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        // Check if we go to the next spike
                        if (numOfSpikes > 0)
                        {
                            follower.followPath(intakeLine1, true);
                            setPathState(2);
                        }
                        else
                        {
                            Park();
                            setPathState(10);
                        }
                    }
                }
                else
                {
                    shooterTimer.resetTimer();
                }
                break;

                // Go back to shooting line
            case 2:
                if (!follower.isBusy()) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(scoreLine1, true);
                    setPathState(3);
                }
                break;

                // Shoot
            case 3:
                if (!follower.isBusy()) {

                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.closeGate();
                        // Check if we go to the next spike
                        if (numOfSpikes > 1)
                        {
                            follower.followPath(lineupIntake2, true);
                            setPathState(4);
                        }
                        else
                        {
                            Park();
                            setPathState(10);
                        }
                    }
                }
                else {
                    shooterTimer.resetTimer();
                }
                break;

                // Intake again
            case 4:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine2, true);
                    setPathState(5);
                }
                break;

                // Go back
            case 5:
                if (!follower.isBusy()) {
                    hw.turret.spinUpFlywheel();
                    follower.followPath(scoreLine2, true);
                    setPathState(6);
                }
                break;

                // Shoot
            case 6:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.closeGate();
                        // Check if we go to the next spike
                        if (numOfSpikes > 2)
                        {
                            follower.followPath(lineupIntake3, true);
                            setPathState(7);
                        }
                        else
                        {
                            Park();
                            setPathState(10);
                        }
                    }
                }
                break;

                // Intake again
            case 7:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(intakeLine3, true);
                    setPathState(8);
                }
                break;

                // Go back
            case 8:
                if (!follower.isBusy()) {
                    hw.turret.spinUpFlywheel();
                    follower.followPath(scoreLine3, true);
                    shooterTimer.resetTimer();
                    setPathState(9);
                }
                break;

                // Shoot
            case 9:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();
                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        Park();
                        setPathState(10);
                    }
                }
                break;

                // Park
            case 10:
                if (!follower.isBusy())
                {
                    setPathState(-1);
                    stop();
                }
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
        autonomousPathRedUpdate();
        telemetry.addLine("RED");

        hw.lights.update();
        hw.turret.update();

        hw.turret.setTarget(follower.getPose(), goalPosition);

        // Constantly save the last known position
        Field.lastKnownPosition = new Pose2D(DistanceUnit.INCH, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.RADIANS, follower.getHeading());

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addLine("Position: " + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.update();

        if (pathState == -1)
        {
            requestOpModeStop();
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
        goalPosition = Field.redGoal;

        // Save the selected alliance side, so TeleOp can read it and automatically load the goal position.
        Field.lastAllianceSide = Field.Side.RED;
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
        follower.setPose(startPose);
        buildPaths();
        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.RED);

        hw.intake.closeGate();

        shotsFired = false;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side currentSide = Field.Side.RED;
    private Field.StartingPosition currentStartingPosition = Field.StartingPosition.NEAR;
    private int numOfSpikes = 3;
    @Override
    public void init_loop()
    {
        hw.lights.update();
        hw.turret.update();

        telemetry.addLine("Press up or down on the D-Pad to select number of spikes");
        telemetry.addLine("Number of spikes selected: " + numOfSpikes);
        if (gamepad1.dpadUpWasPressed() && numOfSpikes != 3)
        {
            numOfSpikes++;
        }
        else if (gamepad1.dpadDownWasPressed() && numOfSpikes != 0)
        {
            numOfSpikes--;
        }
        telemetry.addData("Current Location", follower.getPose());
        telemetry.addLine("Side: " + ((currentSide == Field.Side.RED) ? "Red" : "Blue"));
        telemetry.addLine("Starting Position: " + ((currentStartingPosition == Field.StartingPosition.NEAR) ? "Near" : "Far"));
    }
}