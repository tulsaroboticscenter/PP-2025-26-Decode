package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake
{
    public DcMotorEx innerIntakeMotor = null;
    public DcMotorEx outerIntakeMotor = null;


    public void init(HardwareMap hwMap)
    {
        innerIntakeMotor = hwMap.get(DcMotorEx.class, "innerIntake");
        innerIntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        innerIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        innerIntakeMotor.setPower(0);

        outerIntakeMotor = hwMap.get(DcMotorEx.class, "outerIntake");
        outerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outerIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerIntakeMotor.setPower(0);

        // Direction
        innerIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outerIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intake()
    {
        innerIntakeMotor.setPower(1);
        outerIntakeMotor.setPower(1);
    }
    public void partialIntake() {
        innerIntakeMotor.setPower(0.9);
    }
    public void outtake()
    {
        innerIntakeMotor.setPower(-1);
        outerIntakeMotor.setPower(-1);
    }
    public void stop()
    {
        innerIntakeMotor.setPower(0);
        outerIntakeMotor.setPower(0);
    }

}
