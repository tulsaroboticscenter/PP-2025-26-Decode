package org.firstinspires.ftc.teamcode.Libraries;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class Turret {
    private HWProfile robot = null;
    public Turret(HWProfile myRobot)
    {
        robot = myRobot;
    }

    private PIDFController turretPID = new PIDFController(HWProfile.turretkP, HWProfile.turretkI, HWProfile.turretkD, HWProfile.turretkF, -1600, 1600);

    private static double KpVal = 0.005;
    private static double KdVal = 0.005;
    private double previousDegreesToTarget = 0.0;

    public double turretPPR = 145.1;
    public double turretGearRatio = 4.75;
    public double ticksPerTurretRevolution = turretPPR * turretGearRatio;
    public double ticksToDegreesCoeffecient = 360 / ticksPerTurretRevolution;
    public double ticksToRadiansCoeffecient = (2 * Math.PI) / ticksPerTurretRevolution;

    private final boolean clamped = true;

    public void init()
    {
        turretPID.setTolerance(10);
        turretPID.setTarget(0);
    }

    public void update()
    {
        turretPID.setPIDFCoefficients(HWProfile.turretkP, HWProfile.turretkI, HWProfile.turretkD, HWProfile.turretkF);
        robot.turretRotationMotor.setVelocity(turretPID.calculate(robot.turretRotationMotor.getCurrentPosition()));
    }

    /**
     * Sets the target of the turret for when it is updated with update().
     * @param currentPosition The current position of the robot expressed as a Pose2D object
     * @param targetPosition The position of the desired target expressed as a Pose2D object
     */
    public void setTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        turretPID.setTarget(HeadingToTurretTicks(Targeting.getDegreesToTarget(currentPosition, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(Pose currentPosition, Pose2D targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        turretPID.setTarget(HeadingToTurretTicks(Targeting.getDegreesToTarget(currentPose2D, targetPosition, false), AngleUnit.DEGREES));
    }
    public void setTarget(int tickValue)
    {
        turretPID.setTarget(tickValue);
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




    // A PD-controller for generating a value to turn to a target.
    // in use, the value this function generates replaces the gamepad1.right_stick_x (or turn) value



}
