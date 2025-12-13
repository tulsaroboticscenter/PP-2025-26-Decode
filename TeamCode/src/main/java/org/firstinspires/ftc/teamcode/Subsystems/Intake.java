package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake
{
    private DcMotor intakeMotor = null;
    private Servo gateServo = null;

    private double gateOpenPosition = 0.8;
    private double gateClosedPosition = 0.5;

    public void init(HardwareMap hwMap)
    {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void closedGate()
    {
        gateServo.setPosition(gateClosedPosition);
    }

}
