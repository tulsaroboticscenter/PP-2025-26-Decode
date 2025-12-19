package org.firstinspires.ftc.teamcode.Classes;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class PoseUtils
{
    public static String poseToString(Pose2D pose, DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return "X: " + String.format(Locale.US, "%.2f", pose.getX(distanceUnit)) + " | " +
                "Y: " + String.format(Locale.US, "%.2f", pose.getY(distanceUnit)) + " | " +
                "Heading: " + String.format(Locale.US, "%.2f", pose.getHeading(angleUnit));
    }

    public static String poseToString(Pose pose, DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        // A regular Pose object contains x, y, and heading values that are in millimeters and radians.
        // The values need to be converted to the desired distance unit and angle unit.
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        if (distanceUnit == DistanceUnit.CM)
        {
            x /= 10;
            y /= 10;
        }
        else if (distanceUnit == DistanceUnit.INCH)
        {
            x /= 25.4;
            y /= 25.4;
        }
        else if (distanceUnit == DistanceUnit.METER)
        {
            x /= 1000;
            y /= 1000;
        }

        // Convert Heading to degrees, if it's specified
        if (angleUnit == AngleUnit.DEGREES)
        {
            heading = Math.toDegrees(heading);
        }

        return "X: " + String.format(Locale.US, "%.2f", x) + " | " +
                "Y: " + String.format(Locale.US, "%.2f", y) + " | " +
                "Heading: " + String.format(Locale.US, "%.2f", heading);
    }
}
