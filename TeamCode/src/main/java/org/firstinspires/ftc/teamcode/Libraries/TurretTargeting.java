package org.firstinspires.ftc.teamcode.Libraries;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

//@Config
public class TurretTargeting {

    public HWProfile robot;
    public LinearOpMode opMode;

    public Targeting targeting;

    public TurretTargeting(HWProfile myRobot, Targeting myTargeting)
    {
        robot = myRobot;
        targeting = myTargeting;
    }

    // if the shooter is on the back of the robot instead of the front, set this to true.
    // if the shooter is on the front of the robot, set this to false.
    boolean reversePolarity = true;

    boolean clamped = true;

    public double turretPPR = 145.1;
    public double turretGearRatio = 4.75;

    public double ticksPerTurretRevolution = turretPPR * turretGearRatio;

    public double ticksToDegreesCoeffecient = 360 / ticksPerTurretRevolution;
    public double ticksToRadiansCoeffecient = (2 * Math.PI) / ticksPerTurretRevolution;

    public double getTurretAngle(double turretMotorTicks, AngleUnit angleUnit) {
        double motorRevolutions = turretMotorTicks / ticksPerTurretRevolution;
        double turretAngle = motorRevolutions * 360;

        while (turretAngle > 180) {
            turretAngle -= 360;
        }
        while (turretAngle < -180) {
            turretAngle += 360;
        }

        if (angleUnit == AngleUnit.DEGREES) {
            return turretAngle;
        } else {
            return Math.toRadians(turretAngle);
        }
    }

    public void lockOnTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        robot.turretRotationMotor.setTargetPosition(HeadingToTurretTicks(targeting.getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
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
}