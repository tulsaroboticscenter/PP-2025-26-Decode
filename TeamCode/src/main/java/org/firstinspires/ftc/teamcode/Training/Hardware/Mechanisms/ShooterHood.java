package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterHood {

    public Servo hoodServoLeft = null;
    public Servo hoodServoRight = null;
    static final double HOOD_MIN = 0.0;
    static final double HOOD_MAX = 1.0;
    private double hoodPosition = 0.0;
    static final double hoodMovement = 0.02;

    public enum HoodDirection {
        UP,
        DOWN
    }
    HoodDirection hoodDirection = HoodDirection.UP;

    public void init(HardwareMap hwMap){
        hoodServoLeft = hwMap.get(Servo.class,"hoodServoL");
        hoodServoRight = hwMap.get(Servo.class,"hoodServoR");

        hoodServoRight.setDirection(Servo.Direction.REVERSE);

        hoodDirection = HoodDirection.UP;
        hoodPosition = HOOD_MIN;

        hoodServoLeft.setPosition(hoodPosition);
        hoodServoRight.setPosition(hoodPosition);

    }
    public void moveHood(HoodDirection hd){

        hoodPosition = hoodServoLeft.getPosition();
        hoodPosition = hoodServoRight.getPosition();

        if (hd == HoodDirection.UP){
            hoodPosition += hoodMovement;
        } else{
            hoodPosition -= hoodMovement;
        }
        if (hoodPosition > HOOD_MAX) {
            hoodPosition = HOOD_MAX;

        } else if (hoodPosition < HOOD_MIN) {
            hoodPosition = HOOD_MIN;
        }

        hoodServoRight.setPosition(hoodPosition);
        hoodServoLeft.setPosition(hoodPosition);
    }
}
