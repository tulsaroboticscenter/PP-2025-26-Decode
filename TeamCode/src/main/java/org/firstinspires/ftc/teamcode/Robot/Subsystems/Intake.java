package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake
{
    // --------
    // HARDWARE
    // --------

    public DcMotorEx innerIntakeMotor = null;
    public DcMotorEx outerIntakeMotor = null;
    public Servo gate = null;

    // ---------
    // VARIABLES
    // ---------

    // Switches
    public boolean isIntaking = false;
    public boolean isForceIntaking = false;

    // Amperage Limits
    double innerAmperageLimit = 4.5;
    double outerAmperageLimit = 12.0;

    // --------------
    // INITIALIZATION
    // --------------

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

        gate = hwMap.get(Servo.class, "gate");

        gate.setPosition(0.6);
    }

    // -------
    // METHODS
    // -------

    public void toggle()
    {
        if (isIntaking)
        {
            stop();
            isIntaking = false;
        }
        else
        {
            isIntaking = true;
            innerIntakeMotor.setPower(1);
            outerIntakeMotor.setPower(1);
        }
    }

    public void intake()
    {
        isIntaking = true;
        isForceIntaking = false;
    }
    public void forceIntake() {isForceIntaking = true;}
    public void partialIntake() {
        innerIntakeMotor.setPower(0.9);
    }
    public void outtake() {
        innerIntakeMotor.setPower(-1);
        outerIntakeMotor.setPower(-1);
    }
    public void stop() {
        innerIntakeMotor.setPower(0);
        outerIntakeMotor.setPower(0);
        isForceIntaking = false;
        isIntaking = false;
    }

    public void openGate()
    {
        gate.setPosition(0.28);
    }

    public void closeGate()
    {
        gate.setPosition(0.6);
    }

    public void update() {
        if (isForceIntaking) {
            innerIntakeMotor.setPower(1);
            outerIntakeMotor.setPower(1);
        } else if (isIntaking) {
            if (innerIntakeMotor.getCurrent(CurrentUnit.AMPS) > innerAmperageLimit) {
                innerIntakeMotor.setPower(0);
                if (outerIntakeMotor.getPower() == 0) {
                    isIntaking = false;
                }
            }
            if (outerIntakeMotor.getCurrent(CurrentUnit.AMPS) > outerAmperageLimit) {
                outerIntakeMotor.setPower(0);
                if (innerIntakeMotor.getPower() == 0) {
                    isIntaking = false;
                }
            }
        } else {
            isIntaking = false;
            innerIntakeMotor.setPower(0);
            outerIntakeMotor.setPower(0);
        }
    }
}
