package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Blue Far Leviathan", group = "Autonomous", preselectTeleOp = "tele")
public class blueFarLeviathan extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw = new HardwareManager(hardwareMap);

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    private final Pose scorePose = new Pose(56, 16, 90);
    private final Pose startPose = Field.toPedro(Field.blueSmallZone);
    private final Pose park = new Pose(108, 15.5, Math.toRadians(90));


    public PathChain Movetointake, Movetoshoot, Movetointake2, Movetoshoot2, Park;

    public void buildPaths() {
        Movetointake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.741, 8.562),
                                new Pose(62.270, 39.308),
                                new Pose(17, 36)
                        )
                )
                .setConstantHeadingInterpolation(160)
                .build();

        Movetoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.119, 36.000), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(90))
                .build();

        Movetointake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(61.103, 10.897),
                                new Pose(30, 30)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-100))
                //.setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(8.368, 25),
                                new Pose(8.368, 13)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-100))
                .build();

        Movetoshoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(8.368, 9.535), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        Park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(61.492, 11.286), new Pose(38.335, 16.735))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            /// Spins up flywheel
            case 0:
                hw.turret.spinUpFlywheel();
                shooterTimer.resetTimer();
                setPathState(1);
                break;

            /// Shoot
            case 1:
                if (!follower.isBusy())
                {
                    if (shooterTimer.getElapsedTime() > 2000)
                    {
                        hw.intake.partialIntake();
                        hw.intake.openGate();
                    }

                    if (shooterTimer.getElapsedTime() > 3500)
                    {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        follower.followPath(Movetointake, true);
                        setPathState(2);
                    }
                }
                else
                {
                    shooterTimer.resetTimer();
                }
                break;

            /// intake spike mark, and go back to shoot
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    follower.followPath(Movetoshoot, true);
                    setPathState(3);
                }
                break;

            /// Shoot
            case 3:
                if (!follower.isBusy())
                {
                    if (shooterTimer.getElapsedTime() > 750)
                    {
                        hw.intake.partialIntake();
                        hw.intake.openGate();
                    }

                    if (shooterTimer.getElapsedTime() > 3000)
                    {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        follower.followPath(Movetointake2, true);
                        setPathState(4);
                    }
                }
                else
                {
                    telemetry.addLine("Case 3 -> Busy");
                    shooterTimer.resetTimer();
                }
                break;

            /// Intake again, from the human player station, and move back
            case 4:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(Movetoshoot2, true);
                    shooterTimer.resetTimer();
                    setPathState(5);
                }
                break;

            /// Shoot
            case 5:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy() && shooterTimer.getElapsedTime() > 750) {
                    hw.intake.partialIntake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3500)
                    {
                        hw.intake.closeGate();
                        hw.intake.stop();
                        hw.intake.intake();
                        follower.followPath(Movetointake2, true);
                        setPathState(6);
                    }
                }
                break;
            /// moves to intake again from the human player zone
            case 6:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(Movetoshoot2, true);
                    shooterTimer.resetTimer();
                    setPathState(7);
                }
                break;
            /// shoots for the final time
            case 7:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy() && shooterTimer.getElapsedTime() > 750) {
                    hw.intake.partialIntake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 3500)
                    {
                        hw.intake.closeGate();
                        hw.intake.stop();
                        hw.intake.intake();
                        follower.followPath(Park, true);
                        setPathState(8);
                    }
                }
                break;
            /// park
            case 8:
                if (!follower.isBusy()) {
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
        autonomousPathUpdate();
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
        follower.setPose(startPose);
        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.BLUE);
        hw.intake.closeGate();

        shotsFired = false;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side currentSide = Field.Side.BLUE;
    private Field.StartingPosition currentStartingPosition = Field.StartingPosition.FAR;
    @Override
    public void init_loop()
    {
        hw.lights.update();
        hw.turret.update();

        telemetry.addLine("Side: " + ((currentSide == Field.Side.RED) ? "Red" : "Blue"));
        telemetry.addLine("Starting Position: " + ((currentStartingPosition == Field.StartingPosition.NEAR) ? "Near" : "Far"));
    }
}