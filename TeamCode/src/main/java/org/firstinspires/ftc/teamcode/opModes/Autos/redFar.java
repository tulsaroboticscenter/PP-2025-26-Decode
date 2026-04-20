package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Red Far", group = "Autonomous", preselectTeleOp = "TeleOp")
public class redFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw = new HardwareManager(hardwareMap);

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    private final Pose startPose = Field.toPedro(Field.redSmallZone);
    private final Pose park = new Pose(108, 15.5, Math.toRadians(90));


    private PathChain parkPath, intakeCurve1, scoreLine1, intakeCurve2, scoreLine2, intakeCurve3, scoreLine3;


    public void buildPaths() {
        intakeCurve1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 8.000),
                                new Pose(86.984, 36.584),
                                new Pose(135.438, 35.805)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreLine1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135.438, 35.805), new Pose(83.481, 10.314))
                )
                .setConstantHeadingInterpolation(Math.toRadians(80))
                .build();

        intakeCurve2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.481, 10.314),
                                new Pose(89.903, 65.130),
                                new Pose(135.243, 59.935)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreLine2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135.243, 59.935), new Pose(84.065, 10.314))
                )
                .setConstantHeadingInterpolation(Math.toRadians(80))
                .build();

        intakeCurve3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.065, 10.314),
                                new Pose(87.957, 88.541),
                                new Pose(129.989, 83.870)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreLine3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.989, 83.870), new Pose(84.065, 10.508))
                )
                .setConstantHeadingInterpolation(Math.toRadians(80))
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

            // Shoot
            case 1:
                if (!follower.isBusy())
                {
                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.partialIntake();
                        hw.intake.openGate();
                    }

                    if (shooterTimer.getElapsedTime() > 6000)
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

            // intake
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

                    if (shooterTimer.getElapsedTime() > 4000 && shooterTimer.getElapsedTime() > 3000)
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
                    hw.intake.intake();
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
                if (!follower.isBusy() && shooterTimer.getElapsedTime() > 1500) {
                    hw.intake.partialIntake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.closeGate();
                        hw.intake.stop();
                        // Check if we go to the next spike
                        if (numOfSpikes > 2)
                        {
                            hw.intake.intake();
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
                if (!follower.isBusy() && shooterTimer.getElapsedTime() > 1500) {
                    hw.intake.openGate();
                    hw.intake.partialIntake();
                    if (shooterTimer.getElapsedTime() > 3000)
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
        buildPaths();
        follower.setPose(startPose);
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