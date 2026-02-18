package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.Classes.PoseUtils;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

import java.util.Locale;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpV2", group="Robot")
public class TeleOpV2 extends OpMode
{

    TelemetryManager ptelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private HardwareManager hw = new HardwareManager(hardwareMap);
    private Pose2D goalPosition = null;

    // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
    ElapsedTime totalRuntime = new ElapsedTime();


    private boolean isTargeting = false;
    boolean isIntaking = false;

    Pose2D storedLocation;
    Pose2D startingPosition = null;

    boolean lastSideSeen = false;
    boolean lastPositionSeen = false;

    Field.Side selectedSide = Field.Side.RED;

    boolean endgame = false;

    @Override
    public void init()
    {
        // initialize TeleOp
        hw.initTeleOp(hardwareMap);

        // Initialize Side
        if (Field.lastAllianceSide != null)
        {
            lastSideSeen = true;
            selectedSide = Field.lastAllianceSide;
        }

        if (Field.lastKnownPosition != null)
        {
            lastPositionSeen = true;
            storedLocation = Field.lastKnownPosition;
        }

        // if it knows the last side, but not the last position, default to the far zone of the specific side
        if (Field.lastAllianceSide != null && Field.lastKnownPosition == null)
        {
            storedLocation = ((selectedSide == Field.Side.RED) ? Field.redSmallZone : Field.blueSmallZone);
        }

        // Set Lights to correct color on startup
        hw.lights.setLightColor((selectedSide == Field.Side.RED) ? RGBLightController.RED : RGBLightController.BLUE);
    }

    boolean testing = false;

    Pose2D testingPosition;
    Field.Side testingSide = Field.Side.RED;

    @Override
    public void init_loop()
    {
        hw.updateInitTeleOp();

        telemetry.addLine("Press [Option] to toggle testing mode");
        telemetry.addLine("Testing: " + testing);
        telemetry.addLine();

        // Press Options to toggle testing
        if (gamepad1.optionsWasPressed())
        {
            testing = !testing;
        }

        // If testing mode is on
        if (testing)
        {
            telemetry.addLine("Press [Square] to switch sides");

            testingPosition = (testingSide == Field.Side.RED) ? Field.redSmallZone : Field.blueSmallZone;

            telemetry.addLine("Testing Side: " + ((testingSide == Field.Side.RED) ? "Red" : "Blue"));
            telemetry.addLine("Starting Position: " + PoseUtils.poseToString(testingPosition, DistanceUnit.INCH, AngleUnit.DEGREES));

            // Toggle Testing Side
            if (gamepad1.squareWasPressed())
            {
                testingSide = ((testingSide == Field.Side.RED) ? Field.Side.BLUE : Field.Side.RED);
            }
            hw.lights.setLightColor((testingSide == Field.Side.RED) ? RGBLightController.RED : RGBLightController.BLUE);
        }
        else
        {
            hw.lights.setLightColor((selectedSide == Field.Side.RED) ? RGBLightController.RED : RGBLightController.BLUE);
            // Convey whether positions and sides were found or not.
            if (!lastSideSeen)
            {
                telemetry.addLine("Last Side not found. Defaulting to Red...");
            }
            else
            {
                telemetry.addLine("Last Side Found.");
            }
            if (!lastPositionSeen)
            {
                telemetry.addLine("Last Position not found. Defaulting to Far Zone...");
            }
            else
            {
                telemetry.addLine("Last Position Found.");
            }
        }
    }

    @Override
    public void start()
    {
        hw.turret.init(hardwareMap, true);
        if (testing)
        {
            startingPosition = testingPosition;
            selectedSide = testingSide;
        }
        hw.turret.spinUpFlywheel();

        totalRuntime.reset();

        // Set goal position based on side
        goalPosition = ((selectedSide == Field.Side.RED) ? Field.redGoal : Field.blueGoal);

        hw.pinpoint.setPosition(startingPosition);
    }

    private Pose2D pos;
    @Override
    public void loop()
    {
        // Updating lines
        pos = hw.pinpoint.getPosition();
        hw.updateTeleOp();
        hw.turret.updateFlywheelAndHood(pos, goalPosition);
        hw.drivetrain.fieldOrientedDrive(this, pos, startingPosition.getHeading(AngleUnit.RADIANS), selectedSide);
        hw.lights.setLightColor(((hw.limelight.seesTarget()) ? RGBLightController.GREEN : RGBLightController.RED));

        // Targeting Suite
        if (isTargeting)
        {
            hw.turret.setTarget(pos, goalPosition);
        }
        if (gamepad1.triangleWasPressed())
        {
            isTargeting = !isTargeting;
        }

        // Intaking / Firing Suite
        if (gamepad1.crossWasPressed())
        {
            isIntaking = !isIntaking;
        }
        if (gamepad1.right_trigger > 0.2) {
            hw.intake.openGate();
        } else {
            hw.intake.closeGate();
        }
        if (gamepad1.right_trigger > 0.5) {
            hw.intake.intake();
        }
        if (gamepad1.right_trigger < 0.5) {
            if (isIntaking)
            {
                hw.intake.intake();
            }
            else if (gamepad1.circle)
            {
                hw.intake.outtake();
            }
            else
            {
                hw.intake.stop();
            }
        }

        // Endgame Warning
        if (totalRuntime.seconds() > 100 && !endgame)
        {
            endgame = true;
            gamepad1.setLedColor(1, 1, 1, 1000000000);
            gamepad1.rumbleBlips(4);
        }

        if (hw.limelight.seesTarget() && gamepad1.touchpadWasPressed())
        {
            hw.pinpoint.setPosition(hw.limelight.getRobotPosition(hw.turret.getLocalTurretHeadingDegrees()));
        }

        telemetry.addLine("limelight robot position: " + PoseUtils.poseToString(hw.limelight.getRobotPosition(hw.turret.getLocalTurretHeadingDegrees()), DistanceUnit.INCH, AngleUnit.DEGREES));
        telemetry.update();
    }
}