package org.firstinspires.ftc.teamcode.Classes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class PoseUtils
{
    public static String poseToString(Pose2D pose, DistanceUnit distanceUnit, AngleUnit angleUnit)
    {
        return "X: " + String.format(Locale.US, "%.2f", pose.getX(distanceUnit) + " | Y: " + String.format(Locale.US, "%.2f", pose.getY(distanceUnit) + " | Heading: " + String.format(Locale.US, "%.2f", pose.getHeading(angleUnit))));
    }
}
