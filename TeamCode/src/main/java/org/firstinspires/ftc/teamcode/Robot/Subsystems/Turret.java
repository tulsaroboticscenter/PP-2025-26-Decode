package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private Servo hoodServoL = null;
    private Servo hoodServoR = null;
    private RevTouchSensor turretLimitSwitch = null;

    @Sorter(sort = 0)
    public static double turretkP = 25;
    @Sorter(sort = 1)
    public static double turretkI = 0;
    @Sorter(sort = 2)
    public static double turretkD = 2.5;
    @Sorter(sort = 3)
    public static double turretkF = 0.30;
    @Sorter(sort = 4)
    public static double turretTolerance = 0;

    private PIDFController turretPID = new PIDFController(turretkP, turretkI, turretkD, turretkF, -1600, 1600);
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

    public final double LAUNCHER_LOW_VELOCITY = 1000;
    public final double LAUNCHER_MEDIUM_VELOCITY = 1400;
    public final double LAUNCHER_HIGH_VELOCITY = 1700;

    public final double HOOD_LOW_POSITION = 0;
    public final double HOOD_MEDIUM_POSITION = 0.6;
    public final double HOOD_HIGH_POSITION = 0.9;

    public double velocity = LAUNCHER_LOW_VELOCITY;
    public double hoodTarget = HOOD_LOW_POSITION;
    public boolean isFlywheelSpinning = false;

    // Variables for quadratic equations (y = ax^2 + bx + c)
    public double hoodA = -0.00005222578;
    public double hoodB = 0.0187724131;
    public double hoodC = -0.68522;
    public double flywheelA = -0.0104966341;
    public double flywheelB = 9.274162345;
    public double flywheelC = 635.4612;

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
        turretRotationMotor.setZeroPowerBehavior(BRAKE);

        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        if (!TeleOp)
        {
            hoodServoL.setPosition(1);
            hoodServoR.setPosition(0);
        }
        turretPID.setTolerance(turretTolerance);
        turretPID.setTarget(0);
    }

    public void initHood()
    {
        hoodServoL.setPosition(1);
        hoodServoR.setPosition(0);
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

    public void update()
    {
        turretPID.setPIDFCoefficients(turretkP, turretkI, turretkD, turretkF);
        turretPID.setTolerance(turretTolerance);
        turretRotationMotor.setVelocity(turretPID.calculate(turretRotationMotor.getCurrentPosition()));
        if (isFlywheelSpinning)
        {
            launcherL.setVelocity(velocity);
            launcherR.setVelocity(velocity);
        }
        else
        {
            launcherL.setVelocity(0);
            launcherR.setVelocity(0);
        }
        hoodServoR.setPosition(hoodTarget);
        hoodServoL.setPosition((1 - hoodTarget));
    }

    public void setHoodTarget(double target)
    {
        hoodTarget = target;
    }

    double distanceInches = 0;
    public void updateFlywheelAndHood(Pose2D currentPosition, Pose2D goalPosition)
    {
        distanceInches = getDistanceToTarget(currentPosition, goalPosition);
        hoodTarget = ((hoodA * (distanceInches * distanceInches)) + (hoodB * distanceInches) + hoodC);
        if (distanceInches > 179)
        {
            hoodTarget = 0.9;
        }
        if (hoodTarget > 0.9)
        {
            hoodTarget = 0.9;
        }
        else if (hoodTarget < 0.01)
        {
            hoodTarget = 0.01;
        }

        velocity = ((flywheelA * (distanceInches * distanceInches)) + (flywheelB * distanceInches) + flywheelC);
        if (distanceInches > 441)
        {
            velocity = 2684;
        }
        Range.clip(velocity, 600, 2000);
    }
    public void updateFlywheelAndHood(Pose currentPosition, Pose2D goalPosition)
    {
        distanceInches = getDistanceToTarget(currentPosition, goalPosition);
        hoodTarget = ((hoodA * (distanceInches * distanceInches)) + (hoodB * distanceInches) + hoodC);
        if (distanceInches > 179)
        {
            hoodTarget = 0.9;
        }
        if (hoodTarget > 0.9)
        {
            hoodTarget = 0.9;
        }
        else if (hoodTarget < 0.01)
        {
            hoodTarget = 0.01;
        }

        velocity = ((flywheelA * (distanceInches * distanceInches)) + (flywheelB * distanceInches) + flywheelC);
        if (distanceInches > 441)
        {
            velocity = 2684;
        }
        Range.clip(velocity, 600, 2000);
    }

    /**
     * Sets the target of the turret for when it is updated with update().
     * @param currentPosition The current position of the robot expressed as a Pose2D object
     * @param targetPosition The position of the desired target expressed as a Pose2D object
     */
    public void setTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose2D targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPose2D, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        Pose2D targetPose2D = new Pose2D(DistanceUnit.INCH, targetPosition.getX(), targetPosition.getY(), AngleUnit.RADIANS, targetPosition.getHeading());
        turretPID.setTarget(HeadingToTurretTicks(getDegreesToTarget(currentPose2D, targetPose2D, false), AngleUnit.DEGREES));
    }
    public void setTarget(int tickValue)
    {
        turretPID.setTarget(tickValue);
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

        return startingHue - ((distanceInches / distanceRange) * hueRange);
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
                if (angle > 60)
                {
                    return (int)(60 / ticksToDegreesCoeffecient);
                }
                else if (angle < -150)
                {
                    return (int)(-150 / ticksToDegreesCoeffecient);
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

    public double getTurretHeadingDegrees(double robotHeadingDegrees)
    {
        double turretheading = turretRotationMotor.getCurrentPosition() * ticksToDegreesCoeffecient;
        return robotHeadingDegrees + turretheading;
    }

}
