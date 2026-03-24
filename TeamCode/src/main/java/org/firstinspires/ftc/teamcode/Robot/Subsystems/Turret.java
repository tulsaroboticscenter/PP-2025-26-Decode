package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret
{

    public Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public Servo turretRotationServo1 = null;
    public Servo turretRotationServo2 = null;

    public DcMotorEx launcherL = null;
    public DcMotorEx launcherR = null;

    public Servo gate = null;
    public Servo hoodServo = null;
    public boolean isFlywheelSpinning = false;

    public boolean isTargeting = false;
    public static boolean reversePolarity = true;

    public boolean flywheelOn = false;
    public double velocity = 1300;

    public double hoodTarget = 0;

    public double trOffset = 0;

    // This variable below represents how much the servo has to rotate to get a full 360 degree range of motion
    // The numbers in this variable correspond to the current gear ratio.
    public double servoRange = 1.0 / ((100.0/20.0) * (24.0/95.0));

    public double zeroPosition = 0.5 + trOffset;
    public double leftBound = zeroPosition - (servoRange / 2);
    public double rightBound = zeroPosition + (servoRange / 2);

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        turretRotationServo1 = hwMap.get(Servo.class, "trServo1");
        turretRotationServo2 = hwMap.get(Servo.class, "trServo2");

        launcherL = hwMap.get(DcMotorEx.class, "launcherL");
        launcherR = hwMap.get(DcMotorEx.class, "launcherR");

        gate = hwMap.get(Servo.class, "gate");
        hoodServo = hwMap.get(Servo.class, "hood");

        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherL.setVelocity(0);
        launcherR.setVelocity(0);

        turretRotationServo1.setPosition(zeroPosition);
        turretRotationServo2.setPosition(zeroPosition);

        turretRotationServo1.setDirection(Servo.Direction.REVERSE);
        turretRotationServo2.setDirection(Servo.Direction.REVERSE);

        if (!TeleOp)
            hoodServo.setPosition(0);
    }

    private void setFlywheelMotorVelocity(double velocity)
    {
        launcherL.setVelocity(velocity);
        launcherR.setVelocity(velocity);
    }

    public double HeadingToServoValue(double heading, AngleUnit angleunit)
    {
        double startingHeading = heading;
        if (angleunit == AngleUnit.RADIANS)
        {
            startingHeading *= (180.0 / Math.PI); // if radians, convert to degrees.
        }
        return zeroPosition + ((startingHeading / 360) * servoRange);
    }

    public void spinUpFlywheel(){isFlywheelSpinning = true;}

    public void setTurretPosition(double position)
    {
        turretRotationServo1.setPosition(position);
        turretRotationServo2.setPosition(position);
    }

    public void setTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        currentPose = currentPosition;
        targetPose = targetPosition;
        setTurretPosition(HeadingToServoValue(getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose2D targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        currentPose = currentPose2D;
        targetPose = targetPosition;
        setTurretPosition(HeadingToServoValue(getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        Pose2D targetPose2D = new Pose2D(DistanceUnit.INCH, targetPosition.getX(), targetPosition.getY(), AngleUnit.RADIANS, targetPosition.getHeading());
        currentPose = currentPose2D;
        targetPose = targetPose2D;
        setTurretPosition(HeadingToServoValue(getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(double tickValue)
    {
        setTurretPosition(tickValue);
    }

    public double getAverageFlywheelVelocity()
    {
        return (launcherL.getVelocity() + launcherR.getVelocity()) / 2;
    }

    public void setTargetVelocity(double velocity)
    {
        this.velocity = velocity;
    }

    public void update()
    {
        if (flywheelOn)
        {
            setFlywheelMotorVelocity(velocity);
        }
        else
        {
            setFlywheelMotorVelocity(0);
        }



        hoodServo.setPosition(hoodTarget);
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

    public static double getDegreesToTarget(Pose currentLocation, Pose2D targetLocation, boolean convertToRadians)
    {
        Pose2D currentLocation2D = new Pose2D(DistanceUnit.MM, currentLocation.getX(), currentLocation.getY(), AngleUnit.RADIANS, currentLocation.getHeading());

        // Grabs change in Y and change in X to calculate slope to target
        double deltaY = (targetLocation.getY(DistanceUnit.MM) - currentLocation2D.getY(DistanceUnit.MM));
        double deltaX = (targetLocation.getX(DistanceUnit.MM) - currentLocation2D.getX(DistanceUnit.MM));

        // converts slope into heading to target in radians
        double targetRadians = Math.atan2(deltaY, deltaX);
        double targetDegrees = Math.toDegrees(targetRadians);

        double currentDegrees;
        if (reversePolarity)
        {
            if (currentLocation2D.getHeading(AngleUnit.DEGREES) > 0)
            {
                currentDegrees = currentLocation2D.getHeading(AngleUnit.DEGREES) - 180;
            }
            else
            {
                currentDegrees = currentLocation2D.getHeading(AngleUnit.DEGREES) + 180;
            }
        }
        else
        {
            currentDegrees = currentLocation2D.getHeading(AngleUnit.DEGREES);
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

    public static double getDegreesToTarget(Pose currentLocation, Pose targetLocation, boolean convertToRadians)
    {
        Pose2D currentLocation2D = new Pose2D(DistanceUnit.MM, currentLocation.getX(), currentLocation.getY(), AngleUnit.RADIANS, currentLocation.getHeading());
        Pose2D targetLocation2D = new Pose2D(DistanceUnit.MM, targetLocation.getX(), targetLocation.getY(), AngleUnit.RADIANS, targetLocation.getHeading());

        // Grabs change in Y and change in X to calculate slope to target
        double deltaY = (targetLocation2D.getY(DistanceUnit.MM) - currentLocation2D.getY(DistanceUnit.MM));
        double deltaX = (targetLocation2D.getX(DistanceUnit.MM) - currentLocation2D.getX(DistanceUnit.MM));

        // converts slope into heading to target in radians
        double targetRadians = Math.atan2(deltaY, deltaX);
        double targetDegrees = Math.toDegrees(targetRadians);

        double currentDegrees;
        if (reversePolarity)
        {
            if (currentLocation2D.getHeading(AngleUnit.DEGREES) > 0)
            {
                currentDegrees = currentLocation2D.getHeading(AngleUnit.DEGREES) - 180;
            }
            else
            {
                currentDegrees = currentLocation2D.getHeading(AngleUnit.DEGREES) + 180;
            }
        }
        else
        {
            currentDegrees = currentLocation2D.getHeading(AngleUnit.DEGREES);
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

    public double getCurrentVelocity()
    {
        return velocity;
    }

    public double getHoodTarget() {return hoodTarget;}
}
