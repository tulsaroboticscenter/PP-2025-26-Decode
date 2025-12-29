package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake
{
    public DcMotorEx intakeMotor = null;
    private Servo gateServo = null;

    private double gateOpenPosition = 0.8;
    private double gateClosedPosition = 0.5;

    public void init(HardwareMap hwMap)
    {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        gateServo = hwMap.get(Servo.class, "gate");

    }

    public void intake()
    {
        intakeMotor.setPower(1);
    }
    public void outtake()
    {
        intakeMotor.setPower(-1);
    }
    public void stop()
    {
        intakeMotor.setPower(0);
    }
    public void openGate()
    {
        gateServo.setPosition(gateOpenPosition);
    }
    public void closeGate()
    {
        gateServo.setPosition(gateClosedPosition);
    }

}
