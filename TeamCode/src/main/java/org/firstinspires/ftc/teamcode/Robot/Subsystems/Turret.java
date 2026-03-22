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

    public Servo turretRotationServo1 = null;
    public Servo turretRotationServo2 = null;

    public DcMotorEx launcherL = null;
    public DcMotorEx launcherR = null;

    public Servo gate = null;
    public Servo hoodServo = null;

    public boolean flywheelOn = false;
    public double velocity = 1300;

    public double hoodTarget = 0;

    public double trOffset = 0;

    // This variable below represents how much the servo has to rotate to get a full 360 degree range of motion
    // The numbers in this variable correspond to
    public double servoRange = 1.0 / ((100.0/20.0) * (24.0/95.0));

    public double zeroPosition = 0.5 + trOffset;
    public double leftBound = zeroPosition - (servoRange / 2);
    public double rightBound = zeroPosition + (servoRange / 2);

    public double turretTarget = zeroPosition;

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        turretRotationServo1 = hwMap.get(Servo.class, "trServo1");
        turretRotationServo2 = hwMap.get(Servo.class, "trServo2");

        launcherL = hwMap.get(DcMotorEx.class, "fw1");
        launcherR = hwMap.get(DcMotorEx.class, "fw2");

        gate = hwMap.get(Servo.class, "gate");
        hoodServo = hwMap.get(Servo.class, "hood");

        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherL.setVelocity(0);
        launcherR.setVelocity(0);

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
        return Range.clip(zeroPosition + ((startingHeading / 360) / servoRange), leftBound, rightBound);
    }

    public void setTurretPosition(double position)
    {
        turretRotationServo1.setPosition(position);
        turretRotationServo2.setPosition(position);
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

    public double getCurrentVelocity()
    {
        return velocity;
    }

    public double getHoodTarget() {return hoodTarget;}
}
