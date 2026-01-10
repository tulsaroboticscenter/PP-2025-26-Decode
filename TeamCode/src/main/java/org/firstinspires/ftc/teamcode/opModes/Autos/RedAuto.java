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

@Autonomous(name = "Red Auto", group = "Autonomous", preselectTeleOp = "TeleOp")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private HardwareManager hw = new HardwareManager(hardwareMap);

    private Timer shooterTimer;

    private int pathState;

    public boolean shotsFired;

    public Pose2D goalPosition = null;

    private PathChain scorePreload, intakeLine1, scoreLine1, lineupIntake2, intakeLine2, scoreLine2, lineupIntake3, intakeLine3, scoreLine3;

    // RED NEAR POSES (Not implemented or tested yet)

    private final Pose redNearStartPose = new Pose(120.3, 129.2, Math.toRadians(36));
    private final Pose redNearScorePose = new Pose(93, 84.1, Math.toRadians(90));
    private final Pose redNearIntake1 = new Pose(127.7, 84.1, Math.toRadians(90));
    private final Pose redNearPrepIntake2 = new Pose(101.2, 59.7, Math.toRadians(90));
    private final Pose redNearIntake2 = new Pose(131.4, 59.5, Math.toRadians(90));
    private final Pose redNearPrepIntake3 = new Pose(101.2, 35.8, Math.toRadians(90));
    private final Pose redNearIntake3 = new Pose(131.4, 35.8, Math.toRadians(90));
    private final Pose redNearPark = new Pose(84.3, 103.5);



    private PathChain redNearScorePreload, redNearIntakeLine1, redNearScoreLine1, redNearLineupIntake2, redNearIntakeLine2, redNearScoreLine2, redNearLineupIntake3, redNearIntakeLine3, redNearScoreLine3;


    public void buildPathsRedNear() {
        redNearScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(redNearStartPose, redNearScorePose))
                .setLinearHeadingInterpolation(redNearStartPose.getHeading(), redNearScorePose.getHeading())
                .build();

        redNearIntakeLine1 = follower.pathBuilder()
                .addPath(new BezierLine(redNearScorePose, redNearIntake1))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearScoreLine1 = follower.pathBuilder()
                .addPath(new BezierLine(redNearIntake1, redNearScorePose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearLineupIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(redNearScorePose, redNearPrepIntake2))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearIntakeLine2 = follower.pathBuilder()
                .addPath(new BezierLine(redNearPrepIntake2, redNearIntake2))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearScoreLine2 = follower.pathBuilder()
                .addPath(new BezierLine(redNearIntake2, redNearScorePose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearLineupIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(redNearScorePose, redNearPrepIntake3))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearIntakeLine3 = follower.pathBuilder()
                .addPath(new BezierLine(redNearPrepIntake3, redNearIntake3))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        redNearScoreLine3 = follower.pathBuilder()
                .addPath(new BezierLine(redNearIntake3, redNearPark))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public void autonomousPathRedUpdate() {
        switch (pathState) {
            case 0:
                hw.turret.isFlywheelSpinning = true;
                //hw.turret.setFlywheelVelocity(hw.turret.LAUNCHER_HIGH_VELOCITY);
                hw.turret.launcherL.setVelocity(hw.turret.LAUNCHER_MEDIUM_VELOCITY_AUTO);
                hw.turret.launcherR.setVelocity(hw.turret.LAUNCHER_MEDIUM_VELOCITY_AUTO);
                //hw.turret.setHood(hw.turret.HOOD_MEDIUM_POSITION);
                //hw.turret.hoodServoL.setPosition(hw.turret.HOOD_MEDIUM_POSITION);
                hw.turret.hoodServoR.setPosition(1 - hw.turret.HOOD_MEDIUM_POSITION);
                hw.intake.partialIntake();
                telemetry.addLine("spinning up flywheel-completed");

                follower.followPath(redNearScorePreload, false);
                shooterTimer.resetTimer();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {

                    hw.intake.intake();
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 4000) {
                        hw.intake.closeGate();
                        hw.intake.intake();
                        //hw.turret.haltFlywheel();
                        follower.followPath(redNearIntakeLine1, false);
                        setPathState(2);
                    }

                }
                else {
                    shooterTimer.resetTimer();
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    hw.intake.closeGate();
                    hw.intake.intake();
                    hw.turret.launcherL.setVelocity(hw.turret.LAUNCHER_MEDIUM_VELOCITY_AUTO);
                    hw.turret.launcherR.setVelocity(hw.turret.LAUNCHER_MEDIUM_VELOCITY_AUTO);
                    //hw.turret.setHood(hw.turret.HOOD_MEDIUM_POSITION);
                    //hw.turret.hoodServoL.setPosition(hw.turret.HOOD_MEDIUM_POSITION);
                    hw.turret.hoodServoR.setPosition(1 - hw.turret.HOOD_MEDIUM_POSITION);
                    hw.intake.partialIntake();
                    follower.followPath(redNearScoreLine1, false);
                    setPathState(3);
                }
                break;

            case 3:
                if (follower.isBusy())
                    hw.intake.partialIntake();
                hw.intake.closeGate();
            {

            }
            if (!follower.isBusy()) {

                hw.intake.openGate();

                if (shooterTimer.getElapsedTime() > 4000)
                {
                    //hw.turret.haltFlywheel();
                    hw.intake.closeGate();
                    hw.intake.stop();
                    follower.followPath(redNearLineupIntake2, false);
                    setPathState(4);
                }
            }
            else {
                shooterTimer.resetTimer();
            }
            break;

            case 4:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(redNearIntakeLine2, false);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    hw.turret.spinUpFlywheel();
                    follower.followPath(redNearScoreLine2, false);
                    setPathState(6);
                }
                break;

            case 6:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();

                    if (shooterTimer.getElapsedTime() > 4000)
                    {
                        //hw.turret.haltFlywheel();
                        hw.intake.closeGate();
                        hw.intake.stop();
                        follower.followPath(redNearLineupIntake3, false);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    hw.intake.intake();
                    follower.followPath(redNearIntakeLine3, false);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    hw.turret.spinUpFlywheel();
                    follower.followPath(redNearScoreLine3, false);
                    shooterTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                if (follower.isBusy())
                {
                    shooterTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    hw.intake.openGate();
                    if (shooterTimer.getElapsedTime() > 4000)
                    {
                        setPathState(-1);
                    }
                }
                break;


            case 99:
                hw.turret.isFlywheelSpinning = true;
                //hw.turret.setFlywheelVelocity(hw.turret.LAUNCHER_HIGH_VELOCITY);
                hw.turret.launcherL.setVelocity(hw.turret.LAUNCHER_MEDIUM_VELOCITY);
                hw.turret.launcherR.setVelocity(hw.turret.LAUNCHER_MEDIUM_VELOCITY);
                hw.turret.setHood(hw.turret.HOOD_MEDIUM_POSITION);
                telemetry.addLine("spinning up flywheel-completed");

                //follower.followPath(scorePreload, false);
                shooterTimer.resetTimer();
                setPathState(100);
                break;

            case 100:
                telemetry.addLine("spinning up flywheel-completed");
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
        autonomousPathRedUpdate();
        telemetry.addLine("RED");

        hw.lights.update();
        hw.turret.update();

        hw.turret.setTarget(follower.getPose(), goalPosition);
        //hw.turret.updateFlywheelAndHood(follower.getPose(), goalPosition);

        // Constantly Save the Position
        Field.lastKnownPosition = new Pose2D(DistanceUnit.INCH, follower.getPose().getX(), follower.getPose().getY(), AngleUnit.RADIANS, follower.getHeading());

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addLine("Position: " + PoseUtils.poseToString(follower.getPose(), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.addData("Distance to target:", hw.turret.getDistanceToTarget(follower.getPose(), goalPosition));
        telemetry.update();
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
        buildPathsRedNear();
        follower.setStartingPose(redNearStartPose);
        hw.initPedro(hardwareMap);
        hw.lights.setLightMode(RGBLightController.LEDMode.PULSE_WAKE);
        hw.lights.setLightColor(RGBLightController.BLUE);

        shotsFired = false;

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private Field.Side currentSide = Field.Side.RED;
    private Field.StartingPosition currentStartingPosition = Field.StartingPosition.NEAR;
    @Override
    public void init_loop()
    {
        hw.lights.update();
        hw.turret.update();
        // THIS CURRENTLY DOES NOT DO ANYTHING. THIS AUTO WILL ONLY RUN THE CURRENTLY PROGRAMMED BRANCH

        telemetry.addLine("Side: " + ((currentSide == Field.Side.RED) ? "Red" : "Blue"));
        telemetry.addLine("Starting Position: " + ((currentStartingPosition == Field.StartingPosition.NEAR) ? "Near" : "Far"));
        telemetry.addLine("Press [Square] to switch sides.");
        telemetry.addLine("Press [Triangle] to switch starting positions.");
    }
}