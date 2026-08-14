package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx intakeMotor = null;

    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }
    public void intake(double power) {
        intakeMotor.setPower(power);
    }
}
