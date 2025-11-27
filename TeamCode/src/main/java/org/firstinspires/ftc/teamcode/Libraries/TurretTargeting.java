package org.firstinspires.ftc.teamcode.Libraries;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

import java.io.File;

@Config
public class TurretTargeting {

    public HWProfile robot;
    public LinearOpMode opMode;

    // if the shooter is on the back of the robot instead of the front, set this to true.
    // if the shooter is on the front of the robot, set this to false.
    boolean reversePolarity = true;

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

    public int HeadingToTurretTicks(double angle, AngleUnit angleunit) {
        if (angleunit == AngleUnit.DEGREES) {
            return (int)(angle / ticksToDegreesCoeffecient);
        } else {
            return (int)(angle / ticksToRadiansCoeffecient);
        }
    }


    public TurretTargeting(HWProfile myRobot, LinearOpMode myOpMode)
    {
        robot = myRobot;
        opMode = myOpMode;
    }

    public double getTargetAngle(Pose2D currentLocation, Pose2D targetLocation, AngleUnit angleUnit)
    {
        double deltaY = (targetLocation.getY(DistanceUnit.MM) - currentLocation.getY(DistanceUnit.MM));
        double deltaX = (targetLocation.getX(DistanceUnit.MM) - currentLocation.getX(DistanceUnit.MM));

        double targetRadians = Math.atan2(deltaY, deltaX);

        if (angleUnit == AngleUnit.RADIANS)
        {
            return targetRadians;
        }
        else
        {
            return Math.toDegrees(targetRadians);
        }
    }

    public double getAngleToTarget(Pose2D currentLocation, double currentAngleDegrees, Pose2D targetLocation, AngleUnit angleUnit)
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
            if (currentAngleDegrees > 0)
            {
                currentDegrees = currentAngleDegrees - 180;
            }
            else
            {
                currentDegrees = currentAngleDegrees + 180;
            }
        }
        else
        {
            currentDegrees = currentAngleDegrees;
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

        if (angleUnit == AngleUnit.RADIANS)
        {
            return Math.toRadians(degreesToTarget);
        }
        else
        {
            return degreesToTarget;
        }
    }

    // returns distance from one position to another.
    public double getDistanceToTarget(Pose2D position1, Pose2D position2)
    {
        double deltaY = position2.getY(DistanceUnit.MM) - position1.getY(DistanceUnit.MM);
        double deltaX = position2.getX(DistanceUnit.MM) - position1.getX(DistanceUnit.MM);

        double distanceMM = Math.sqrt(Math.pow(deltaX, 2) - Math.pow(deltaY, 2));

        return distanceMM;
    }


    public void writeToFile (double headingValue, String fileName)
    {
        File headingValueAfterAuto = AppUtil.getInstance().getSettingsFile(fileName);
        ReadWriteFile.writeFile(headingValueAfterAuto, String.valueOf(headingValue));
    }

    public double readFromFile (String fromFileName)
    {
        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newly declared filename.
        // See Note 4 above.
        double myNumber = Double.parseDouble(ReadWriteFile.readFile(myFileName).trim());

        return myNumber;       // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()

}

