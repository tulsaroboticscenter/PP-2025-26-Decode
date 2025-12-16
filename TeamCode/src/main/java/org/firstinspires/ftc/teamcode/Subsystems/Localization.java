package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.goBilda.GoBildaPinpointDriver;

public class Localization
{
    private GoBildaPinpointDriver pinpoint = null;

    public static double leadMagnitudeMultiplier = 1;

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(76.2, -190.5); // x: 3in y: -7.5in
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public void setPosition(Pose2D position)
    {
        pinpoint.setPosition(position);
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

    public Pose2D getLeadPose(Pose2D pos)
    {
        double[] velocityVector = getVelocityVector();
        double magnitude = velocityVector[0] * leadMagnitudeMultiplier;
        double theta = velocityVector[1];
        return new Pose2D(DistanceUnit.MM,
                pos.getX(DistanceUnit.MM) + (magnitude * Math.cos(theta)),
                pos.getY(DistanceUnit.MM) + (magnitude * Math.sin(theta)),
                AngleUnit.RADIANS, 0);
    }


}
