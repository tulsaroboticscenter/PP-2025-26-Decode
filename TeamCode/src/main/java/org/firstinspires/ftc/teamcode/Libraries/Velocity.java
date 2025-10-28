package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

import java.lang.Math;

public class Velocity {

    public HWProfile robot;

    public LinearOpMode opMode;



    private double magnitude = 0.0;
    private double theta = 0.0;

    private double inverseTheta = 0.0;

    private double velX;
    private double velY;

    // controls the strength of how much the robot leads when targeting.
    public static double leadCoefficient = 1;

    private Pose2D leadCoord = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

    public ElapsedTime velocityTimer = new ElapsedTime();


    public Velocity(HWProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
    }

    public void resetTimer() {
        velocityTimer.reset();
    }

    // called every cycle
    // this function monitors the robot's velocity and makes a
    public void Monitor(double timeIntervalSeconds, Pose2D targetPosition) {
        if (velocityTimer.seconds() > timeIntervalSeconds) {

            // pinpoint.getVel returns velocity in mm/sec
            velX = robot.pinpoint.getVelX();
            velY = robot.pinpoint.getVelY();
            magnitude = Math.sqrt(Math.pow(velX, 2) + Math.pow(velY, 2));

            // theta is the heading of the vector
            theta = Math.atan2(velY, velX);

            if (theta > 0) {
                inverseTheta = theta - Math.PI;
            } else {
                inverseTheta = theta + Math.PI;
            }

            leadCoord = new Pose2D(DistanceUnit.MM,
                    ((magnitude * leadCoefficient) * Math.cos(inverseTheta)), // Right here, we find the end of the vector.
                    ((magnitude * leadCoefficient) * Math.sin(inverseTheta)),
                    AngleUnit.RADIANS, inverseTheta); // we store the direction of our vector just in case we need it later.
        }
    }

    public double getMagnitude() {
        return magnitude;
    }

    public Pose2D getLeadCoord() {
        return leadCoord;
    }

}

