package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;

public class Turret
{

    public Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public Servo turretRotationServo1 = null;
    public Servo turretRotationServo2 = null;

    public AnalogInput turretRotationEncoder = null;

    public DcMotorEx launcherL = null;
    public DcMotorEx launcherR = null;

    public Servo hoodServo = null;

    public boolean isFlywheelSpinning = false;
    public boolean isTargeting = false;
    // Turret facing backwards initially instead of forwards?
    public static boolean reversePolarity = true;
    public static boolean isManuallySetting = false;
    public boolean isLeading = false;

    public double targetVelocity = 2400; // Refactored from velocity to targetVelocity for clarity
    public double hoodTarget = 0;
    public double lastHeading;
    public double currentHeading = 0;
    public double continuousHeading = 0;

    // PIDF change-detection trackers
    private double lastKP = -1;
    private double lastKI = -1;
    private double lastKD = -1;

    // Flywheel Velocity PIDF Values
    @Sorter(sort = 5)
    public static double flywheelkP = 60;
    @Sorter(sort = 6)
    public static double flywheelkI = 5;
    @Sorter(sort = 7)
    public static double flywheelkD = 25;
    @Sorter(sort = 8)
    public static double flywheelkF = 0;
    @Sorter(sort = 9)
    public static double flywheelTolerance = 5;
    @Sorter(sort = 10)
    public static double redAimOffsetDegrees = 0.0;
    @Sorter(sort = 11)
    public static double blueAimOffsetDegrees = 0.0;

    public double flywheelA = -0.00228584;
    public double flywheelB = 7.6883;
    public double flywheelC = 1088.81631;

    public double hoodA = 9.33907e-10;
    public double hoodB = -0.00000675112;
    public double hoodC = 0.0160059;
    public double hoodD = -11.6092;

    public double trOffset = 0;

    //public final double TURRET_PER_SERVO = (100.0/20.0) * (24.0/95.0);

    // The actual, physical range of the servo
    // Taken by comparing two photos of the servos' extreme points
    public double MAX_ANGLE = 400.78;
    public double zeroPosition = 0.5 + trOffset;
    public final double TURRET_OFFSET_MM = -26.16; // Offset from center of robot to turret

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        turretRotationServo1 = hwMap.get(Servo.class, "trServo1"); //
        if (turretRotationServo1 == null)
        {
            throw new IllegalStateException("turretRotationalServo1 not found in hardware map. Check robot configuration.");
        }

        turretRotationServo2 = hwMap.get(Servo.class, "trServo2"); //
        if (turretRotationServo2 == null)
        {
            throw new IllegalStateException("turretRotationalServo2 motor not found in hardware map. Check robot configuration.");
        }

        launcherL = hwMap.get(DcMotorEx.class, "launcherL"); //
        if (launcherL == null)
        {
            throw new IllegalStateException("launcherL motor not found in hardware map. Check robot configuration.");
        }

        launcherR = hwMap.get(DcMotorEx.class, "launcherR"); //
        if (launcherR == null)
        {
            throw new IllegalStateException("launcherR motor not found in hardware map. Check robot configuration.");
        }

        hoodServo = hwMap.get(Servo.class, "hood"); //
        if (hoodServo == null)
        {
            throw new IllegalStateException("hoodServo not found in hardware map. Check robot configuration.");
        }


        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherL.setVelocity(0);
        launcherR.setVelocity(0);

        turretRotationServo1.setDirection(Servo.Direction.REVERSE);
        turretRotationServo2.setDirection(Servo.Direction.REVERSE);

        if (!TeleOp) {
            hoodServo.setPosition(0);
            turretRotationServo1.setPosition(zeroPosition);
            turretRotationServo2.setPosition(zeroPosition);
        }

    }

    private void setFlywheelMotorVelocity(double velocity)
    {
        launcherL.setVelocity(velocity);
        launcherR.setVelocity(velocity);
    }

    public void toggleFlywheel()
    {
        isFlywheelSpinning = !isFlywheelSpinning;
    }

    public void stopFlywheel()
    {
        isFlywheelSpinning = false;
    }

    public double HeadingToServoValue(double heading, AngleUnit angleunit)
    {
        if(Double.isNaN(heading)) return zeroPosition;
        double startingHeading = heading;
        if (angleunit == AngleUnit.RADIANS)
            startingHeading = Math.toDegrees(startingHeading);

        // flip + to - if rotating wrong way
        return MathFunctions.clamp(zeroPosition - (startingHeading / MAX_ANGLE), 0, 1);
    }

    public void spinUpFlywheel(){isFlywheelSpinning = true;}

    public void setTurretPosition(double position)
    {
        if(Double.isNaN(position)) return;

            //        {
//            // temporary debug
//            // guard against NaN from any caller
//            throw new RuntimeException("NaN servo position.  current Heading=" + currentHeading
//            + " continuousHeading=" + continuousHeading
//            + "hoodTarget= " + hoodTarget
//            +"zeroPosition=" + zeroPosition
//            +" trOffset=" + trOffset);
//        }
        turretRotationServo1.setPosition(MathFunctions.clamp(position, 0, 1));
        turretRotationServo2.setPosition(MathFunctions.clamp(position, 0, 1));
    }

    public void setTarget(Pose2D currentPosition, Pose2D targetPosition)
    {
        isManuallySetting = false;
        currentPose = currentPosition;
        targetPose = targetPosition;
    }
    public void setTarget(Pose currentPosition, Pose2D targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        setTarget(currentPose2D, targetPosition);
    }
    public void setTarget(Pose currentPosition, Pose targetPosition)
    {
        Pose2D currentPose2D = new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading());
        Pose2D targetPose2D = new Pose2D(DistanceUnit.INCH, targetPosition.getX(), targetPosition.getY(), AngleUnit.RADIANS, targetPosition.getHeading());
        setTarget(currentPose2D, targetPose2D);
    }
    public void setTarget(double angle, AngleUnit angleUnit)
    {
        isManuallySetting = true;
        setTurretPosition(HeadingToServoValue(angle, angleUnit));
    }
    public void setTarget(double tickValue)
    {
        setTurretPosition(tickValue);
    }

    /**
     * Offsets a robot pose to the location of its turret.
     * @param pose The position of the robot.
     * @return A corrected pose adjusted for the turret's position
     */
    public Pose2D offsetPoseToTurret(Pose2D pose)
    {
        double theta = pose.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.MM, pose.getX(DistanceUnit.MM) + (TURRET_OFFSET_MM * Math.cos(theta)), pose.getY(DistanceUnit.MM) + (TURRET_OFFSET_MM * Math.sin(theta)), AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
    }

    double distanceInches = 0;
    public void updateFlywheelAndHood(Pose2D currentPosition, Pose2D goalPosition)
    {
        double tempTarget = 0;
//        if (isLeading)
//        {
//            distanceInches = getDistanceToTarget(currentPosition, lastLeadPose);
//        }
//        else
//        {
            distanceInches = getDistanceToTarget(currentPosition, goalPosition);
        //}


        // THIS is where you compute your regression for the flywheel. Adjust this to match the equation you came up with
        // Quadratic Example: ((flywheelA * Math.pow(distanceInches, 2)) + (flywheelB * distanceInches) + flywheelC)

        targetVelocity = ((flywheelA * Math.pow(distanceInches, 2)) + (flywheelB * distanceInches) + flywheelC);
        targetVelocity = MathFunctions.clamp(targetVelocity, 1300, 2500);

        double averageVelocity = (launcherL.getVelocity() + launcherR.getVelocity()) / 2;

        // THIS is where you compute your regression for the hood. Note that x is now the flywheel velocity, not the distance.
        // Cubic Example: ((hoodA * Math.pow(averageVelocity, 3)) + (hoodB * Math.pow(averageVelocity, 2)) + (hoodC * averageVelocity) + hoodD)

//        tempTarget = ((hoodA * Math.pow(targetVelocity, 3)) +
//                (hoodB * Math.pow(targetVelocity, 2)) +
//                (hoodC * targetVelocity) + hoodD);

        tempTarget = ((hoodA * Math.pow(averageVelocity, 3)) +
        (hoodB * Math.pow(averageVelocity, 2)) +
        (hoodC * averageVelocity) + hoodD);

        hoodTarget = MathFunctions.clamp(tempTarget, 0, 0.87);
    }

    public void updateFlywheelAndHood(Pose currentPosition, Pose2D goalPosition)
    {
        updateFlywheelAndHood(new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading()), goalPosition);
    }

    public double getAverageFlywheelVelocity()
    {
        return (launcherL.getVelocity() + launcherR.getVelocity()) / 2;
    }

    public void setTargetVelocity(double velocity)
    {
        this.targetVelocity = velocity;
    }

    /**
     * New update() method added by CTS on 4/21/2026
     *
     * Notes:
     * There are two separate problems here working against each other:
     * Problem A — the delta guard threshold is wrong. It guards against jumps > 270° but getDegreesToTarget()
     * returns values in [-180, 180]. The maximum real delta between two loop cycles is maybe 5-10° for a
     * fast-spinning robot. A jump of 270° will never be a real movement — it's always a wrap artifact. The
     * threshold should be 180°, not 270°.
     *
     * Problem B — the boundary clamp teleports the servo. If continuousHeading drifts to +201° and the
     * clamp fires, it instantly becomes −159°. That's a 360° servo jump in one loop cycle — exactly the
     * snap you're seeing near the physical limit.
     *
     *
     * continuousHeading is now a pure running total of rotation. It can technically exceed ±MAX_ANGLE/2 —
     * that's fine, it just means the target is outside physical reach. The servo gets clamped cleanly to
     * the edge of its range and holds there without snapping.
     *
     */

    public void update() {

        // PIDF update (unchanged)
        if (flywheelkP != lastKP || flywheelkI != lastKI || flywheelkD != lastKD) {
            launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));
            launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));
            lastKP = flywheelkP; lastKI = flywheelkI; lastKD = flywheelkD;
        }

        // Flywheel
        if (isFlywheelSpinning) setFlywheelMotorVelocity(targetVelocity);
        else                    setFlywheelMotorVelocity(0);


//        if(Double.isNaN(hoodTarget)) return;
//        {
//            // temporary debug
//            // guard against NaN from any caller
//            throw new RuntimeException("NaN servo position.  current Heading=" + currentHeading
//                    + " continuousHeading=" + continuousHeading
//                    + "hoodTarget= " + hoodTarget
//                    +"zeroPosition=" + zeroPosition
//                    +" trOffset=" + trOffset);
//        }

        if (Double.isNaN(hoodTarget)) hoodTarget = 0;
        hoodServo.setPosition(MathFunctions.clamp(hoodTarget, 0, 0.87));

        // ------ NOTICE -------------------------
        // This section of code is to utilize the full ~400 degrees of rotation of the turret,
        // not to limit the turret to only 360 degrees. This prevents the back-and-forth snap
        // of the turret when going from one degree extreme to the other.

        // We first grab the robot-relative degrees to target.
        currentHeading = getDegreesToTarget(offsetPoseToTurret(currentPose), targetPose, false);
        if(Field.lastAllianceSide == Field.Side.RED)
            currentHeading += redAimOffsetDegrees; //apply aim trim
        else currentHeading += blueAimOffsetDegrees; //apply aim trim

//        lastHeading = getDegreesToTarget(currentPose, targetPose, false);

        // Then we find the difference from the last cycle to the current cycle.
        double delta = currentHeading - lastHeading;

        // If the pinpoint has flipped from 180 to -180, or vice versa,
        // we add or subtract 360 to ignore the pinpoint's IMU limits.
        if (delta > 180)
            delta -= 360;
        else if (delta < -180)
            delta += 360;

        // Then we add it to our continuous heading.

        continuousHeading += delta;
        // And update the last heading.
        lastHeading = currentHeading;

        // THEN, we check if the heading we give to the servos are beyond their physical limits
        // If they are, we flip it back.
        if (continuousHeading > (MAX_ANGLE / 2))
            continuousHeading -= 360;
        else if (continuousHeading < -(MAX_ANGLE / 2))
            continuousHeading += 360;

        // Then we tell the servos to run to the calculated position.
        if (isTargeting && !isManuallySetting)
            // Check if the polarity of the turret is flipped. If it is, then assign the polar opposite value.
            setTurretPosition(HeadingToServoValue(continuousHeading, AngleUnit.DEGREES));
        else if (!isManuallySetting)
            setTurretPosition(HeadingToServoValue(0, AngleUnit.DEGREES));

        // In this way, we give the turret an extra ~40 degrees of freedom, increasing efficiency
    }

/*  ===> Old update() commented out by CTS on 4/21/2026

    public void update()
    {
        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));
        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(flywheelkP, flywheelkI, flywheelkD, flywheelkF));

        if (isFlywheelSpinning)
        {
            setFlywheelMotorVelocity(velocity);
        }
        else
        {
            setFlywheelMotorVelocity(0);
        }
        hoodServo.setPosition(MathFunctions.clamp(hoodTarget, 0, 0.87));

        currentHeading = getDegreesToTarget(offsetPoseToTurret(currentPose), targetPose, false);

        double delta = currentHeading - lastHeading;

        if (delta > 270)
            delta -= 360;
        else if (delta < -270)
            delta += 360;

        continuousHeading += delta;
        lastHeading = currentHeading;

        if (continuousHeading > (MAX_ANGLE / 2))
        {
            continuousHeading -= 360;
        }
        else if (continuousHeading < -(MAX_ANGLE / 2))
        {
            continuousHeading += 360;
        }

        if (isTargeting && !isManuallySetting)
            setTurretPosition(HeadingToServoValue(continuousHeading, AngleUnit.DEGREES));
        else if (!isManuallySetting)
            setTurretPosition(HeadingToServoValue(0, AngleUnit.DEGREES));
    }
*/
    public void incrementHood (double value)
    {
        hoodTarget = MathFunctions.clamp(hoodTarget + value, 0, 0.87);
    }

    public double getServoHeading(AngleUnit angleUnit)
    {
        double headingDegrees = (turretRotationEncoder.getVoltage() - 0.5) * MAX_ANGLE;
        return (angleUnit == AngleUnit.RADIANS) ? Math.toRadians(headingDegrees) : headingDegrees;
    }

    public void incrementFlywheel (int value)
    {
        targetVelocity += value;
    }

    public double getDistanceToTarget(Pose2D Pos1, Pose2D Pos2)
    {
        double deltaY = Pos2.getY(DistanceUnit.INCH) - Pos1.getY(DistanceUnit.INCH);
        double deltaX = Pos2.getX(DistanceUnit.INCH) - Pos1.getX(DistanceUnit.INCH);

        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
    public double getDistanceToTarget(Pose currentPosition, Pose2D targetPosition)
    {
        return getDistanceToTarget(new Pose2D(DistanceUnit.INCH, currentPosition.getX(), currentPosition.getY(), AngleUnit.RADIANS, currentPosition.getHeading()), targetPosition);
    }

    public static double getHeading(Pose pose1, Pose2D pose2)
    {
        return Math.atan2(pose2.getY(DistanceUnit.INCH) - pose1.getY(),  pose2.getX(DistanceUnit.INCH) - pose1.getX());
    }

    public static Pose toPose(Pose2D pose2D)
    {
        return new Pose(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
    }

    public static Pose2D toPose2D(Pose pose)
    {
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

//    public static double getDegreesToTarget(Pose2D currentLocation, Pose2D targetLocation, boolean convertToRadians)
//    {
//        // Grabs change in Y and change in X to calculate slope to target
//        double deltaY = (targetLocation.getY(DistanceUnit.MM) - currentLocation.getY(DistanceUnit.MM));
//        double deltaX = (targetLocation.getX(DistanceUnit.MM) - currentLocation.getX(DistanceUnit.MM));
//
//        // If robot is exactly on teh goal, deltaX and deltaY are both 0 -> atan2(0,0) = NaN.
//        // Return 0 as a safe fallback - turret holds current position.
//        if(deltaX == 0 && deltaY ==0) return 0;
//
//        // converts slope into heading to target in radians
//        double targetRadians = Math.atan2(deltaY, deltaX);
//        double targetDegrees = Math.toDegrees(targetRadians);
//
//        double currentDegrees;
//        if (reversePolarity)
//        {
//            if (currentLocation.getHeading(AngleUnit.DEGREES) > 0)
//            {
//                currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES) - 180;
//            }
//            else
//            {
//                currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES) + 180;
//            }
//        }
//        else
//        {
//            currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES);
//        }
//
//        // this value indicates where the target is relative to the robot's heading
//        // if the value is negative, the target is to the left
//        // if the value is positive, the target is to the right
//        double degreesToTarget = targetDegrees - currentDegrees;
//
//        // Sometimes the value of degreesToTarget is greater than 180 degrees, which is never possible.
//        // This normalizes the value to be between -180 and 180.
//        while (degreesToTarget > 180) {
//            degreesToTarget -= 360;
//        }
//        while (degreesToTarget < -180) {
//            degreesToTarget += 360;
//        }
//
//        if (convertToRadians)
//        {
//            return Math.toRadians(degreesToTarget);
//        }
//        else
//        {
//            return degreesToTarget;
//        }
//    }

    public static double getDegreesToTarget(Pose2D currentLocation, Pose2D targetLocation, boolean convertToRadians)
    {
        // Grabs change in Y and change in X to calculate slope to target
        double deltaY = (targetLocation.getY(DistanceUnit.MM) - currentLocation.getY(DistanceUnit.MM));
        double deltaX = (targetLocation.getX(DistanceUnit.MM) - currentLocation.getX(DistanceUnit.MM));

        // If robot is exactly on the goal, deltaX and deltaY are both 0 -> atan2(0,0) = NaN.
        // Return 0 as a safe fallback - turret holds current position.
        if (deltaX == 0 && deltaY == 0) return 0;

        // converts slope into heading to target in radians
        double targetRadians = Math.atan2(deltaY, deltaX);
        double targetDegrees = Math.toDegrees(targetRadians);

        double currentDegrees;
        if (reversePolarity)
        {
            currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES) + 180;
            if (currentDegrees > 180) currentDegrees -= 360;
        }
        else
        {
            currentDegrees = currentLocation.getHeading(AngleUnit.DEGREES);
        }

        // this value indicates where the target is relative to the robot's heading
        // if the value is negative, the target is to the left
        // if the value is positive, the target is to the right
        double degreesToTarget = targetDegrees - currentDegrees;

        // Normalizes the value to be between -180 and 180.
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

    public static double getDegreesToTarget(Pose currentLocation, Pose2D targetLocation, boolean convertToRadians)
    {
        return getDegreesToTarget(toPose2D(currentLocation), targetLocation, convertToRadians);
    }

    public static double getDegreesToTarget(Pose currentLocation, Pose targetLocation, boolean convertToRadians)
    {
        return getDegreesToTarget(toPose2D(currentLocation), toPose2D(targetLocation), convertToRadians);
    }

    public double getTargetVelocity()
    {
        return targetVelocity;
    }

    public double getHoodTarget() {return hoodTarget;}


    public void seedHeading(Pose2D currentPosition, Pose2D targetPosition) {
        currentPose = currentPosition;
        targetPose  = targetPosition;
        lastHeading = getDegreesToTarget(offsetPoseToTurret(currentPose), targetPose, false);
        if (Double.isNaN(lastHeading)) lastHeading = 0;
        continuousHeading = lastHeading;
    }
}
