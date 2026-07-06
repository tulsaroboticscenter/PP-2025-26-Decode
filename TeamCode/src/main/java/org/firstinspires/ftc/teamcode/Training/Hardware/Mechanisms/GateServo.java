package org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GateServo {

    public Servo gateServo = null;
    static final double GATE_MIN = 0.0;
    static final double GATE_MAX = 1.0;
    private double gatePosition = 0.0;
    static final double gateMovement = 0.02;

    public enum GateDirection {
        OPEN,
        CLOSED
    }
    GateDirection gateDirection = GateDirection.CLOSED;

    public void init(HardwareMap hwMap){
        gateServo = hwMap.get(Servo.class,"hoodServoR");

        gateServo.setDirection(Servo.Direction.REVERSE);

        gateDirection = GateDirection.CLOSED;
        gatePosition = GATE_MIN;

        gateServo.setPosition(gatePosition);

    }
    public void moveGate(GateDirection hd){

        gatePosition = gateServo.getPosition();

        if (hd == GateDirection.OPEN){
            gatePosition += gateMovement;
        } else{
            gatePosition -= gateMovement;
        }
        if (gatePosition > GATE_MAX) {
            gatePosition = GATE_MAX;

        } else if (gatePosition < GATE_MIN) {
            gatePosition = GATE_MIN;
        }

        gateServo.setPosition(gatePosition);
    }
}
