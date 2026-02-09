package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.PIDFController;
import org.firstinspires.ftc.teamcode.Classes.RGBLightController;


@Configurable
public class Turret
{
    public DcMotorEx turretRotationMotor = null; // trMotor
    public DcMotorEx launcherL = null;
    public DcMotorEx launcherR = null;
    public Servo hoodServoL = null;
    public Servo hoodServoR = null;
    private RevTouchSensor turretLimitSwitch = null;

    // Turret Rotation PIDF Values
    @Sorter(sort = 0)
    public static double turretkP = 10;
    @Sorter(sort = 1)
    public static double turretkI = 0;
    @Sorter(sort = 2)
    public static double turretkD = 0.25;
    @Sorter(sort = 3)
    public static double turretkF = 0;
    @Sorter(sort = 4)
    public static double turretTolerance = 0;

    // Flywheel Velocity PIDF Values
    @Sorter(sort = 5)
    public static double flywheelkP = 60;
    @Sorter(sort = 6)
    public static double flywheelkI = 5;
    @Sorter(sort = 7)
    public static double flywheelkD = 25;
    @Sorter(sort = 8)
    public static double flywheelkF = 0;
    @Sorter(sort = 9)
    public static double flywheelTolerance = 5;

    private PIDFController turretPID = new PIDFController(turretkP, turretkI, turretkD, turretkF, -2500, 2500);

    private double previousDegreesToTarget = 0.0;

    public double turretPPR = 384.5;
    public double turretGearRatio = 4.75;
    public double ticksPerTurretRevolution = turretPPR * turretGearRatio;
    public double ticksToDegreesCoeffecient = 360 / ticksPerTurretRevolution;
    public double ticksToRadiansCoeffecient = (2 * Math.PI) / ticksPerTurretRevolution;

    private enum turretStates {
        HIGH,
        MEDIUM,
        LOW
    }

    private turretStates currentStatus = turretStates.LOW;

    private final boolean clamped = true;

    public static boolean reversePolarity = false;

    public boolean isLeading = true;
    private Pose2D lastLeadPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

    @Sorter(sort = 10)
    public static double leadMagnitudeMultiplier = 0.9;

    public final double LAUNCHER_LOW_VELOCITY = 1000;
    public final double LAUNCHER_MEDIUM_VELOCITY = 1400;
    public final double LAUNCHER_MEDIUM_VELOCITY_AUTO = 2000;
    public final double LAUNCHER_HIGH_VELOCITY = 1700;

    /**
    public final double SCORE_HEIGHT = 28.0;
    public final double SCORE_ANGLE = Math.toRadians(-30);
    public final double PASS_THROUGH_POINT_RADIUS = 5;
     **/

    public final double HOOD_LOW_POSITION = 0;
    public final double HOOD_MEDIUM_POSITION = 0.6;
    public final double HOOD_HIGH_POSITION = 0.9;

    public double velocity = LAUNCHER_LOW_VELOCITY;
    public double hoodTarget = 0.275;
    public boolean isFlywheelSpinning = false;

    // Variables for regressions
    public double hoodA = 2.3787e-9;
    public double hoodB = -0.0000136258;
    public double hoodC = 0.0261005;
    public double hoodD = -15.84687;

    public double flywheelA = 0.0152667;
    public double flywheelB = 3.35652;
    public double flywheelC = 1186.6063;

    private Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    private Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);


    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        turretRotationMotor = hwMap.get(DcMotorEx.class, "trMotor");
        launcherL = hwMap.get(DcMotorEx.class, "launcherL");
        launcherR = hwMap.get(DcMotorEx.class, "launcherR");
        hoodServoL = hwMap.get(Servo.class, "hoodServoL");
        hoodServoR = hwMap.get(Servo.class, "hoodServoR");
        turretLimitSwitch = hwMap.get(RevTouchSensor.class, "turretLimitSwitch");

        turretRotationMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        turretRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (!TeleOp)
        {
            turretRotationMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        turretRotationMotor.setPower(0);
        turretRotationMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretRotationMotor.setVelocity(0);
        turretRotationMotor.setZeroPowerBehavior(FLOAT);

        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!TeleOp)
        {
            hoodServoL.setPosition(0.275);
            hoodServoR.setPosition(1 - 0.275);
        }
        else
        {
            hoodServoL.setPosition(hoodServoL.getPosition());
            hoodServoR.setPosition(hoodServoR.getPosition());
        }
        turretPID.setTolerance(turretTolerance);
        turretPID.setTarget(0);

        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));
        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));
    }

    public void initHood()
    {
        hoodServoR.setPosition(0);
        hoodServoL.setPosition(1);
    }

    public void setHood(double position)
    {
        hoodServoL.setPosition(1 - position);
        hoodServoR.setPosition(position);
    }

    public void setFlywheelVelocity(double velocity)
    {
        this.velocity = velocity;
    }

    public void incrementVelocity(double incrementValue)
    {
        velocity += incrementValue;
    }
    public void incrementHoodTarget(double incrementValue)
    {
        hoodTarget += incrementValue;
        if (hoodTarget > 1)
        {
            hoodTarget = 1;
        }
        else if (hoodTarget < 0)
        {
            hoodTarget = 0;
        }
    }

    public void ToggleFlywheel()
    {
        isFlywheelSpinning = !isFlywheelSpinning;
    }

    public void spinUpFlywheel()
    {
        isFlywheelSpinning = true;
    }

    public void haltFlywheel()
    {
        isFlywheelSpinning = false;
    }

    public Pose2D getLeadPose(Pose2D goalPose, double velX, double velY)
    {
        double magnitude = Math.sqrt(Math.pow(velX, 2) + Math.pow(velY, 2));
        double theta = Math.atan2(velY, velX);

        if (theta > 0)
        {
            theta -= Math.PI;
        }
        else if (theta < 0)
        {
            theta += Math.PI;
        }

        magnitude *= leadMagnitudeMultiplier;
        return new Pose2D(DistanceUnit.MM,
                goalPose.getX(DistanceUnit.MM) + (magnitude * Math.cos(theta)),
                goalPose.getY(DistanceUnit.MM) + (magnitude * Math.sin(theta)),
                AngleUnit.RADIANS, 0);
    }

    public void update()
    {
        turretPID.setPIDFCoefficients(turretkP, turretkI, turretkD, turretkF);
        turretPID.setTolerance(turretTolerance);
        turretRotationMotor.setVelocity(turretPID.calculate(turretRotationMotor.getCurrentPosition()));

        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));
        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));

        if (isFlywheelSpinning)
        {
            launcherL.setVelocity(velocity);
            launcherR.setVelocity(velocity);
        }
        else
        {
            launcherR.setVelocity(0);
            launcherL.setVelocity(0);
        }
        hoodTarget = Range.clip(hoodTarget, 0.275, 1);

        hoodServoR.setPosition(hoodTarget);
        hoodServoL.setPosition((1 - hoodTarget));
    }

    public void setFlywheelPower(double power)
    {
        launcherL.setPower(power);
        launcherR.setPower(power);
    }

    public void setHoodTarget(double target)
    {
        hoodTarget = target;
    }

    double distanceInches = 0;
    public void updateFlywheelAndHood(Pose2D currentPosition, Pose2D goalPosition)
    {
        double tempTarget = 0;
        if (isLeading)
        {
            distanceInches = getDistanceToTarget(currentPosition, lastLeadPose);
        }
        else
        {
            distanceInches = getDistanceToTarget(currentPosition, goalPosition);
        }


        // THIS is where you compute your regression for the flywheel. Adjust this to match the equation you came up with

        // Quadratic Example: ((flywheelA * Math.pow(distanceInches, 2)) + (flywheelB * distanceInches) + flywheelC)

        velocity = ((flywheelA * Math.pow(distanceInches, 2)) + (flywheelB * distanceInches) + flywheelC);
        Range.clip(velocity, 600, 2500);

        double averageVelocity = (launcherL.getVelocity() + launcherR.getVelocity()) / 2;


        // THIS is where you compute your regression for the hood. Note that x is now the flywheel velocity, not the distance.

        // Cubic Example: ((hoodA * Math.pow(averageVelocity, 3)) + (hoodB * Math.pow(averageVelocity, 2)) + (hoodC * averageVelocity) + hoodD)

        tempTarget = ((hoodA * Math.pow(averageVelocity, 3)) + (hoodB * Math.pow(averageVelocity, 2)) + (hoodC * averageVelocity) + hoodD);
        if (tempTarget > 1)
        {
            hoodTarget = 1;
        }
        else if (tempTarget < 0.275)
        {
            hoodTarget = 0.275;
        }
        else
        {
            hoodTarget = tempTarget;
        }
        hoodTarget = Range.clip(hoodTarget, 0.275, 1);
    }

    public double getAverageFlywheelVelocity()
    {
        return (launcherL.getVelocity() + launcherR.getVelocity()) / 2;
    }

    public void updateFlywheelAndHood(Pose currentPosition, Pose2D goalPosition)
    {
        double tempTarget = 0;
        if (isLeading)
        {
            distanceInches = getDistanceToTarget(currentPosition, lastLeadPose);
        }
        else
        {
            distanceInches = getDistanceToTarget(currentPosition, goalPosition);
        }

        // THIS is where you compute your regression for the flywheel. Adjust this to match the equation you came up with

        // Quadratic Example: ((flywheelA * Math.pow(distanceInches, 2)) + (flywheelB * distanceInches) + flywheelC)

        velocity = ((flywheelA * Math.pow(distanceInches, 2)) + (flywheelB * distanceInches) + flywheelC);
        Range.clip(velocity, 1200, 2500);

        double averageVelocity = (launcherL.getVelocity() + launcherR.getVelocity()) / 2;

        // THIS is where you compute your regression for the hood. Note that x is now the flywheel velocity, not the distance.

        // Cubic Example: ((hoodA * Math.pow(averageVelocity, 3)) + (hoodB * Math.pow(averageVelocity, 2)) + (hoodC * averageVelocity) + hoodD)

        tempTarget = ((hoodA * Math.pow(averageVelocity, 3)) + (hoodB * Math.pow(averageVelocity, 2)) + (hoodC * averageVelocity) + hoodD);

        if (tempTarget > 1)
        {
            hoodTarget = 1;
        }
        else if (tempTarget < 0.275)
        {
            hoodTarget = 0.275;
        }
        else
        {
            hoodTarget = tempTarget;
        }
        hoodTarget = Range.clip(hoodTarget, 0.275, 1);
    }


    public void manuallySetFlywheelAndHood(double velocity, double hoodTarget)
    {
//        double tempTarget = 0;
//        if (isLeading)
//        {
//            distanceInches = getDistanceToTarget(currentPosition, lastLeadPose);
//        }
//        else
//        {
//            distanceInches = getDistanceToTarget(currentPosition, goalPosition) - fudgeFactor;
//        }

//        velocity = ((flywheelA * Math.pow(distanceInches, 3)) + (flywheelB * Math.pow(distanceInches, 2)) + flywheelC);
//        if (distanceInches > 441)
//        {
//            velocity = 2684;
//        }

        this.velocity = velocity;
        Range.clip(velocity, 1300, 2000);

//        double averageVelocity = (launcherL.getVelocity() + launcherR.getVelocity()) / 2;

//        tempTarget = ((hoodA * Math.pow(averageVelocity, 3)) + (hoodB * Math.pow(averageVelocity, 2)) + (hoodC * averageVelocity) + hoodD);
//        if (tempTarget > 0.9)
//        {
//            hoodTarget = 0.9;
//        }
//        else if (tempTarget < 0.01)
//        {
//            hoodTarget = 0.01;
//        }
//        else
//        {
//            hoodTarget = tempTarget;
//        }
        this.hoodTarget = hoodTarget;
        hoodTarget = Range.clip(hoodTarget, 0.01, 0.9);
    }

    /**
     * Sets the target of the turret for when it is updated with update().
     * @param currentPosition The current position of the robot expressed as a Pose2D object
     * @param targetPosition The position of the desired target expressed as a Pose2D object
     */
    public void setTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        currentPose = currentPosition;
        targetPose = targetPosition;
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose2D targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        currentPose = currentPose2D;
        targetPose = targetPosition;
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPose2D, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        Pose2D targetPose2D = new Pose2D(DistanceUnit.INCH, targetPosition.getX(), targetPosition.getY(), AngleUnit.RADIANS, targetPosition.getHeading());
        currentPose = currentPose2D;
        targetPose = targetPose2D;
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPose2D, targetPose2D, false), AngleUnit.DEGREES));
    }
    public void setTarget(int tickValue)
    {
        turretPID.setTarget(tickValue);
    }

    public void setLeadTarget(Pose2D currentPosition, Pose2D targetPosition, double velocityX, double velocityY)
    {
        lastLeadPose = getLeadPose(targetPosition, velocityX, velocityY);
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPosition, lastLeadPose, false), AngleUnit.DEGREES));
    }

    public double getDistanceToTarget(Pose2D Pos1, Pose2D Pos2)
    {
        double deltaY = Pos2.getY(DistanceUnit.INCH) - Pos1.getY(DistanceUnit.INCH);
        double deltaX = Pos2.getX(DistanceUnit.INCH) - Pos1.getX(DistanceUnit.INCH);

        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
    public double getDistanceToTarget(Pose currentPosition, Pose2D targetPosition)
    {
        double deltaY = targetPosition.getY(DistanceUnit.INCH) - currentPosition.getY();
        double deltaX = targetPosition.getX(DistanceUnit.INCH) - currentPosition.getX();

        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public double getHueFromDistance(Pose2D currentPosition, Pose2D targetPosition)
    {
        double distanceInches = getDistanceToTarget(currentPosition, targetPosition);
        double distanceRange = 150.0;
        double hueRange = RGBLightController.GREEN - RGBLightController.RED;
        double startingHue = RGBLightController.GREEN;

        if (distanceInches > distanceRange)
        {
            distanceInches = distanceRange;
        }

        return Range.clip((startingHue - ((distanceInches / distanceRange) * hueRange)), 0, 1);
    }

    public double getCurrentVelocity()
    {
        return velocity;
    }

    public double getHoodTarget() {return hoodTarget;}
    public int HeadingToTurretTicks(double angle, AngleUnit angleunit) {
        if (clamped)
        {
            if (angleunit == AngleUnit.DEGREES)
            {
                if (angle > 90)
                {
                    return (int)(90 / ticksToDegreesCoeffecient);
                }
                else if (angle < -180)
                {
                    return (int)(-180 / ticksToDegreesCoeffecient);
                }
                else
                {
                    return (int)(angle / ticksToDegreesCoeffecient);
                }
            }
            else
            {
                if (angle > (Math.PI / 2))
                {
                    return (int)((Math.PI / 2) / ticksToRadiansCoeffecient);
                }
                else if (angle < -(Math.PI / 2))
                {
                    return (int)((Math.PI / 2) / ticksToRadiansCoeffecient);
                }
                else
                {
                    return (int)(angle / ticksToRadiansCoeffecient);
                }
            }
        }
        else
        {
            if (angleunit == AngleUnit.DEGREES)
            {
                return (int)(angle / ticksToDegreesCoeffecient);
            }
            else
            {
                return (int)(angle / ticksToRadiansCoeffecient);
            }
        }
    }

    public void zeroHood()
    {
        hoodServoR.setPosition(0);
    }

    public static double getDegreesToTarget(Pose2D currentLocation, Pose2D targetLocation, boolean convertToRadians)
    {
        // Grabs change in Y and change in X to calculate slope to target
        double deltaY = (targetLocation.getY(DistanceUnit.MM) - currentLocation.getY(DistanceUnit.MM));
        double deltaX = (targetLocation.getX(DistanceUnit.MM) - currentLocation.getX(DistanceUnit.MM));

        // converts slope into heading to target in radians
        double targetRadians = Math.atan2(deltaY, deltaX);
        double targetDegrees = Math.toDegrees(targetRadians);

        double currentDegrees;
        if (reversePolarity)
        {
            if (currentLocation.getHeading(AngleUnit.DEGREES) > 0)
            {
                currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES) - 180;
            }
            else
            {
                currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES) + 180;
            }
        }
        else
        {
            currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES);
        }

        // this value indicates where the target is relative to the robot's heading
        // if the value is negative, the target is to the left
        // if the value is positive, the target is to the right
        double degreesToTarget = currentDegrees - targetDegrees;

        // Sometimes the value of degreesToTarget is greater than 180 degrees, which is never possible.
        // This normalizes the value to be between -180 and 180.
        while (degreesToTarget > 180) {
            degreesToTarget -= 360;
        }
        while (degreesToTarget < -180) {
            degreesToTarget += 360;
        }

        if (convertToRadians)
        {
            return Math.toRadians(degreesToTarget);
        }
        else
        {
            return degreesToTarget;
        }
    }


    public double inchPerSecToFlywheelTicks(double velocity)
    {
        double ticksPerRotation = 28;
        double flywheelCircumferenceInches = 8.90530201016; // (72 * pi) / 25.4
        double ticksPerInch = ticksPerRotation / flywheelCircumferenceInches;
        return ticksPerInch * velocity;
    }
    public double degreesToHoodValue(double degrees)
    {
        return Range.clip(0.275 + ((degrees - 32) / 34.4827586207), 0.275, 1);
    }

    public double getGlobalTurretHeadingDegrees(double robotHeadingDegrees)
    {
        double turretheading = turretRotationMotor.getCurrentPosition() * ticksToDegreesCoeffecient;
        return robotHeadingDegrees + turretheading;
    }

    public double getLocalTurretHeadingDegrees()
    {
        return turretRotationMotor.getCurrentPosition() * ticksToDegreesCoeffecient;
    }

}
