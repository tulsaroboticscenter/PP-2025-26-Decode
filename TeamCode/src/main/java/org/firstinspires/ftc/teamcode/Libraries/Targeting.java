package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import java.lang.Math;

import java.io.File;

public class Targeting {

    public HWProfile robot;
    public LinearOpMode opMode;

    // if the shooter is on the back of the robot instead of the front, set this to true.
    // if the shooter is on the front of the robot, set this to false.
    boolean reversePolarity = true;


    public Targeting(HWProfile myRobot, LinearOpMode myOpMode)
    {
        robot = myRobot;
        opMode = myOpMode;
    }

    public void rotateToTarget(Pose2D targetLocation, double tolerance) {

        ElapsedTime correctionTimer = new ElapsedTime();
        double correctionCheckDurationSeconds = 0.5;

        while (opMode.opModeIsActive()) {

            double turn = getTargetingRotationPowerPD(robot.pinpoint.getPosition(), targetLocation, tolerance, robot.pdTimer, true);

            robot.leftFrontDrive.setPower(turn);
            robot.leftBackDrive.setPower(turn);
            robot.rightFrontDrive.setPower(-turn);
            robot.rightBackDrive.setPower(-turn);
            if (getDegreesToTarget(robot.pinpoint.getPosition(), targetLocation, false) > tolerance) {
                // If the robot is in tolerance, check if it has been in tolerance for correctionCheckDurationSeconds
                if (correctionTimer.seconds() > correctionCheckDurationSeconds) {
                    // if it is, then break out of the while loop.
                    break;
                }
            } else {
                // if the robot is not in tolerance, reset the correction timer.
                correctionTimer.reset();
            }
        }

    }

    public double getTargetDegree(Pose2D currentLocation, Pose2D targetLocation, boolean convertToRadians)
    {
        double deltaY = (targetLocation.getY(DistanceUnit.MM) - currentLocation.getY(DistanceUnit.MM));
        double deltaX = (targetLocation.getX(DistanceUnit.MM) - currentLocation.getX(DistanceUnit.MM));

        double targetRadians = Math.atan2(deltaY, deltaX);

        if (convertToRadians)
        {
            return targetRadians;
        }
        else
        {
            return Math.toDegrees(targetRadians);
        }
    }

    public double getDegreesToTarget(Pose2D currentLocation, Pose2D targetLocation, boolean convertToRadians)
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


    private double previousDegreesToTarget = 0.0;

    // A PD-controller for generating a value to turn to a target.
    // in use, the value this function generates replaces the gamepad1.right_stick_x (or turn) value
    public double getTargetingRotationPowerPD(Pose2D currentLocation, Pose2D targetLocation, double tolerance, ElapsedTime pdTimer, boolean reversePolarity) {
        double currentDegreesToTarget = getDegreesToTarget(currentLocation, targetLocation, false);

        if (Math.abs(currentDegreesToTarget) <= tolerance)
        {
            previousDegreesToTarget = currentDegreesToTarget; // Keep it updated even when stopped
            return 0; // Within tolerance
        }

        double dt = pdTimer.seconds(); // Time since last calculation
        pdTimer.reset(); // Reset for next cycle

        // Prevent division by zero or huge derivative spike on first run or after long pause
        if (dt == 0 || dt > 0.5)
        { // If dt is too large, it might indicate a resume, skip D for one cycle
            dt = 0.02; // Assume a typical loop time, or simply skip D-term for this iteration
            // or set derivative to 0 for this cycle.
        }

        // Proportional value
        // When tuning, start with this value and start small, e.g., 0.01 to 0.05,
        // then double until you see oscillation (back and forth movement).
        double Kp = 0.01;
        double proportionalPower = Kp * currentDegreesToTarget;

        // Derivative value
        // Tune this after Kp. start small, (e.g., 0.001 to 0.05) then increase until oscillations stop
        double Kd = 0.01;
        double errorRateOfChange = (currentDegreesToTarget - previousDegreesToTarget) / dt;
        double derivativePower = Kd * errorRateOfChange;

        // Update for next iteration
        previousDegreesToTarget = currentDegreesToTarget;

        // Total Power
        // If currentDegreesToTarget is positive (target to the left), and it's decreasing
        // (meaning errorRateOfChange is negative), we are moving towards target.
        // derivativePower will be negative (Kd * negative_roc).
        // This negative derivativePower will subtract from positive proportionalPower, slowing it down.
        double totalPower = proportionalPower + derivativePower;

        // Minimum power application (handle with care with PD)
        double minPowerAbs = 0.15; // Tune this
        if (Math.abs(totalPower) > 0.001 && Math.abs(totalPower) < minPowerAbs && Math.abs(currentDegreesToTarget) > tolerance)
        {
            totalPower = Math.copySign(minPowerAbs, totalPower);
        }

        // Clamp power
        return Range.clip(totalPower, -1.0, 1.0);
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

