package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Classes.Field;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

public class Pinpoint
{
    public GoBildaPinpointDriver pinpoint = null;

    public static double leadMagnitudeMultiplier = 1;

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        //while (!(pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY))
        //{
            // Do nothing
        //}
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(76.2, -190.5); // x: 3in y: -7.5in
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setYawScalar(1); // flip gyroscope polarity
        pinpoint.recalibrateIMU();
    }

    public void setPosition(Pose2D position)
    {
        pinpoint.setPosition(position);
    }

    public Pose2D getPosition()
    {
        return pinpoint.getPosition();
    }

    // returns data for a velocity vector in polar coordinates and mm/s (r, theta)
    public double[] getVelocityVector()
    {
        double xVel = pinpoint.getVelX();
        double yVel = pinpoint.getVelY();
        double magnitude = Math.sqrt(Math.pow(xVel, 2) + Math.pow(yVel, 2));
        double theta = Math.atan2(yVel, xVel);
        return new double[]{magnitude, theta};
    }

    public double getVelX()
    {
        return pinpoint.getVelX();
    }
    public double getVelY()
    {
        return pinpoint.getVelY();
    }

    public void update()
    {
        pinpoint.update();
    }

    public Pose2D getLeadPose(Pose2D startingGoalPos)
    {
        double[] velocityVector = getVelocityVector();
        double magnitude = velocityVector[0] * leadMagnitudeMultiplier;
        double theta = velocityVector[1];
        return new Pose2D(DistanceUnit.MM,
                startingGoalPos.getX(DistanceUnit.MM) + (magnitude * Math.cos(theta)),
                startingGoalPos.getY(DistanceUnit.MM) + (magnitude * Math.sin(theta)),
                AngleUnit.RADIANS, 0);
    }

    public void resetPosition(Field.Side side)
    {
        pinpoint.setPosition((side == Field.Side.BLUE) ? Field.blueTouchingGoalFacingToward : Field.redTouchingGoalFacingToward);
    }


}
