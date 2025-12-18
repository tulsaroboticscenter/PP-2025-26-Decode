package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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

import java.awt.font.NumericShaper;

public class Turret
{
    private DcMotorEx turretRotationMotor = null; // trMotor
    private DcMotorEx launcherL = null;
    private DcMotorEx launcherR = null;
    private Servo hoodServoL = null;
    private Servo hoodServoR = null;
    private RevTouchSensor turretLimitSwitch = null;

    public static double turretkP = 25;
    public static double turretkI = 0;
    public static double turretkD = 2.5;
    public static double turretkF = 5;
    public static double turretTolerance = 10;

    private PIDFController turretPID = new PIDFController(turretkP, turretkI, turretkD, turretkF, -1600, 1600);

    private static double KpVal = 0.005;
    private static double KdVal = 0.005;
    private double previousDegreesToTarget = 0.0;

    public double turretPPR = 145.1;
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
    public double hoodC = 0.68522;
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
        setHood(hoodTarget);
    }

    public void updateFlywheelAndHood(Pose2D currentPosition, Pose2D goalPosition)
    {
        double distanceInches = getDistanceToTarget(currentPosition, goalPosition);
        hoodTarget = ((hoodA * (distanceInches * distanceInches)) + (hoodB * distanceInches) + hoodC);
        if (distanceInches > 179)
        {
            hoodTarget = 1;
        }
        Range.clip(hoodTarget, 0, 0.9);

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

    public double getDistanceToTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        double deltaY = targetPosition.getY(DistanceUnit.INCH) - currentPosition.getY(DistanceUnit.INCH);
        double deltaX = targetPosition.getX(DistanceUnit.INCH) - currentPosition.getX(DistanceUnit.INCH);

        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    public double getCurrentVelocity()
    {
        return velocity;
    }

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

}
