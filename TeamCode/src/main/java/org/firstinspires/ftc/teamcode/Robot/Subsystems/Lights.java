package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Classes.RGBLightController;

public class Lights
{
    private Servo rgb1 = null;
    private Servo rgb2 = null;

    private RGBLightController light1 = null;
    private RGBLightController light2 = null;

    public void init(HardwareMap hwMap, boolean TeleOp)
    {
        rgb1 = hwMap.get(Servo.class, "rgb1");
        rgb2 = hwMap.get(Servo.class, "rgb2");

        light1 = new RGBLightController(rgb1);
        light2 = new RGBLightController(rgb2);
    }

    public void setLightMode(RGBLightController.LEDMode mode)
    {
        light1.setMode(mode);
        light2.setMode(mode);
    }
    public void setLightColor(double color)
    {
        light1.setColor(color);
        light2.setColor(color);
    }
    public void update()
    {
        light1.update();
        light2.update();
    }

    public RGBLightController.LEDMode getLightMode()
    {
        return light1.getCurrentMode();
    }
    public double getLightColor() {return light1.getCurrentColor();}
}
