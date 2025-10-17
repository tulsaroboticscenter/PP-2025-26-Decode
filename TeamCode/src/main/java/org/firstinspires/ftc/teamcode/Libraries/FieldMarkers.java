package org.firstinspires.ftc.teamcode.Libraries;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class FieldMarkers {

    /**

     This file is a library of points of interest and start positions for reference in autonomous and TeleOp.

     **/

        // Points of Interest

        public Pose2D blueGoal = new Pose2D(DistanceUnit.INCH, -60, 60, AngleUnit.DEGREES, 0);
        public Pose2D redGoal = new Pose2D(DistanceUnit.INCH, 60, 60, AngleUnit.DEGREES, 0);
        public Pose2D redHumanPlayerZone = new Pose2D(DistanceUnit.INCH, -54, -54, AngleUnit.DEGREES, 0);
        public Pose2D blueHumanPlayerZone = new Pose2D(DistanceUnit.INCH, 54, -54, AngleUnit.DEGREES, 0);
        public Pose2D redBase = new Pose2D(DistanceUnit.INCH, -21, -36, AngleUnit.DEGREES, 0);
        public Pose2D blueBase = new Pose2D(DistanceUnit.INCH, 21, -36, AngleUnit.DEGREES, 0);
        public Pose2D center = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        public Pose2D blueDriver = new Pose2D(DistanceUnit.INCH, 80, -48, AngleUnit.DEGREES, 180);
        public Pose2D redDriver = new Pose2D(DistanceUnit.INCH, -80, -48, AngleUnit.DEGREES, 0);



        // Starting Positions

        public Pose2D blueSmallZone = new Pose2D(DistanceUnit.INCH, 12, -60, AngleUnit.DEGREES, 90);
        public Pose2D redSmallZone = new Pose2D(DistanceUnit.INCH, -12, -60, AngleUnit.DEGREES, 90);
        public Pose2D blueTouchingGoal = new Pose2D(DistanceUnit.INCH, -48, 48, AngleUnit.DEGREES, -36.678);
        public Pose2D redTouchingGoal = new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, -143.332);


}
