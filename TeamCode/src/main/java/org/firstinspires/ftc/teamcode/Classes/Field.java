package org.firstinspires.ftc.teamcode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//@Config
public class Field {

    public enum Side
    {
        RED,
        BLUE
    }

    public enum StartingPosition
    {
        NEAR,
        FAR
    }

    /**

     This file is a library of points of interest and start positions for reference in autonomous and TeleOp.
     This Library also stores the last side and position to maintain localization from Auto to TeleOp.

     **/

    public static Side lastAllianceSide = null;
    public static Pose2D lastKnownPosition = null;

    // Points of Interest
    public static final Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, 5, 129, AngleUnit.DEGREES, 0);
    public static final Pose2D redGoal = new Pose2D(DistanceUnit.INCH, 139, 137, AngleUnit.DEGREES, 0);
    public static final Pose2D blueGoalLocal = new Pose2D(DistanceUnit.INCH, 100, -100, AngleUnit.DEGREES, 0);
    public static final Pose2D redGoalLocal = new Pose2D(DistanceUnit.INCH, -46, -70, AngleUnit.DEGREES, 0);

    public static final Pose2D blueHumanPlayerZone = new Pose2D(DistanceUnit.INCH, 136.5, 7.7, AngleUnit.DEGREES, 90);
    public static final Pose2D redHumanPlayerZone = new Pose2D(DistanceUnit.INCH, 7.5, 7.7, AngleUnit.DEGREES, 90);
    public static final Pose2D redBase = new Pose2D(DistanceUnit.INCH, -21, -36, AngleUnit.DEGREES, 0);
    public static final Pose2D blueBase = new Pose2D(DistanceUnit.INCH, 21, -36, AngleUnit.DEGREES, 0);
    public static final Pose2D center = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);


    public static final Pose2D bluePlayer = new Pose2D(DistanceUnit.INCH, 168, 48, AngleUnit.DEGREES, 180);
    public static final Pose2D redPlayer = new Pose2D(DistanceUnit.INCH, -24, 48, AngleUnit.DEGREES, 0);

    // Starting Positions
    public static final Pose2D blueSmallZone = new Pose2D(DistanceUnit.INCH, 51, 7.7, AngleUnit.DEGREES, 0);
    public static final Pose2D redSmallZone = new Pose2D(DistanceUnit.INCH, 84, 7.7, AngleUnit.DEGREES, 0);
    public static final Pose2D blueSmallZoneMovingForward = new Pose2D(DistanceUnit.INCH, 12, -60, AngleUnit.DEGREES, 0);
    public static final Pose2D blueTouchingGoalFacingAway = new Pose2D(DistanceUnit.INCH, -48, 48, AngleUnit.DEGREES, -36.678);
    public static final Pose2D redTouchingGoalFacingAway = new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, -143.332);

    public static final Pose2D blueTouchingGoalFacingToward = new Pose2D(DistanceUnit.INCH, 24, 120, AngleUnit.DEGREES, 143.332);

    public static final Pose2D redTouchingGoalFacingToward = new Pose2D(DistanceUnit.INCH, 120, 120, AngleUnit.DEGREES, 36.678);

    // Conversion methods for Pedro
    public static Pose toPedro(Pose2D pose2D)
    {
        return new Pose(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
    }
    public static Pose2D toPedro2D(Pose2D pose2D)
    {
        return new Pose2D(DistanceUnit.INCH, pose2D.getX(DistanceUnit.INCH) + 72, pose2D.getY(DistanceUnit.INCH) + 72, AngleUnit.DEGREES, pose2D.getHeading(AngleUnit.RADIANS));
    }

}
