package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Blue Far", group = "Autonomous", preselectTeleOp = "TeleOp")
public class blueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw = new HardwareManager(hardwareMap);

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    private final Pose startPose = new Pose(60, 9, Math.toRadians(90));
    private final Pose park = new Pose(36, 10, Math.toRadians(90));


    private PathChain parkPath, intakeCurve1, scoreLine1, intakeCurve2, scoreLine2, intakeCurve3, scoreLine3;


    public void buildPaths() {
        intakeCurve1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.883, 8.932),
                                new Pose(43.990, 34.922),
                                new Pose(39.738, 36.097),
                                new Pose(18.874, 35.534)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scoreLine1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.874, 35.534),

                                new Pose(56.311, 16.485)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeCurve2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.311, 16.485),
                                new Pose(44.087, 62.529),
                                new Pose(38.417, 60.733),
                                new Pose(19.049, 59.893)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scoreLine2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.049, 59.893),

                                new Pose(56.466, 16.340)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intakeCurve3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.466, 16.340),
                                new Pose(48.801, 83.680),
                                new Pose(41.947, 84.748),
                                new Pose(19.000, 84.010)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scoreLine3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.000, 84.010),

                                new Pose(56.689, 16.350)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void Park()
    {
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), park))
                .setConstantHeadingInterpolation(park.getHeading())
                .build();

        follower.followPath(parkPath);
    }

    public void autonomousPathRedUpdate() {
        switch (pathState) {
            // Back up to shoot
            case 0:
                hw.turret.spinUpFlywheel();
                shooterTimer.resetTimer();
                setPathState(1);
                break;

                // Intake
            case 1:
                if (!follower.isBusy())
                {
                    if (shooterTimer.getElapsedTime() > 1000)
                    {
                        hw.intake.intake();
                        hw.intake.openGate();
                    }

                    if (shooterTimer.getElapsedTime() > 4000)
                    {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        // Check if we go to the next spike
                        if (numOfSpikes > 0)
                        {
                            follower.followPath(intakeCurve1, true);
                            setPathState(2);
                        }
                        else
                        {
                            Park();
                            setPathState(8);
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
                        hw.intake.intake();
                        // Check if we go to the next spike
                        if (numOfSpikes > 1)
                        {
                            follower.followPath(intakeCurve2, true);
                            setPathState(4);
                        }
                        else
                        {
                            Park();
                            setPathState(8);
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
                    hw.intake.stop();
                    follower.followPath(scoreLine2, true);
                    shooterTimer.resetTimer();
                    setPathState(5);
                }
                break;

                // Shoot
            case 5:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.closeGate();
                        hw.intake.stop();
                        // Check if we go to the next spike
                        if (numOfSpikes > 2)
                        {
                            follower.followPath(intakeCurve3, true);
                            setPathState(6);
                        }
                        else
                        {
                            Park();
                            setPathState(8);
                        }
                    }
                }
                break;

                // Go back
            case 6:
                if (!follower.isBusy()) {
                    //hw.turret.spinUpFlywheel();
                    follower.followPath(scoreLine3, true);
                    shooterTimer.resetTimer();
                    setPathState(7);
                }
                break;

                // Shoot
            case 7:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();
                    if (shooterTimer.getElapsedTime() > 4000)
                    {
                        Park();
                        setPathState(8);
                    }
                }
                break;

                // Park
            case 8:
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
//        hw.turret.updateFlywheelAndHood(follower.getPose(), goalPosition);
        hw.turret.manuallySetFlywheelAndHood(2200, 0.9);

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
        goalPosition = Field.blueGoal;

        // Save the selected alliance side, so TeleOp can read it and automatically load the goal position.
        Field.lastAllianceSide = Field.Side.BLUE;
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
        hw.intake.closeGate();

        shotsFired = false;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side currentSide = Field.Side.BLUE;
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

        telemetry.addLine("Side: " + ((currentSide == Field.Side.RED) ? "Red" : "Blue"));
        telemetry.addLine("Starting Position: " + ((currentStartingPosition == Field.StartingPosition.NEAR) ? "Near" : "Far"));
    }
}