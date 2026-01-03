package org.firstinspires.ftc.teamcode.Classes;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RGBLightController {

    public static final double RED = 0.28;
    public static final double BLUE = 0.611;
    public static final double GREEN = 0.5;

    private final Servo light;
    private LEDMode currentMode = LEDMode.SOLID;
    private double color = 0;
    private long modeStartTime;

    public enum LEDMode {
        SOLID,
        PULSE,
        RAINBOW,
        FLASH,
        WAKE,
        PULSE_WAKE
    }

    // Call once in init
    public RGBLightController(Servo light) {
        this.light = light;
        this.modeStartTime = System.currentTimeMillis();
    }

    // Switch to a new effect
    public void setMode(LEDMode mode) {
        currentMode = mode;
        modeStartTime = System.currentTimeMillis();
    }

    public void setColor(double newColor)
    {
        color = newColor;
    }

    public LEDMode getCurrentMode()
    {
        return currentMode;
    }
    public double getCurrentColor() {return color;}

    // Call this continuously in loop()
    public void update()
    {
        long t = System.currentTimeMillis() - modeStartTime;

        color = Range.clip(color, 0, 1);

        switch (currentMode)
        {
            case SOLID:
                light.setPosition(color);
                break;

            case PULSE:
                // slow breathing effect (0.3-0.7)
                double pulse = 0.5 + 0.22 * Math.sin(t / 300.0);
                pulse = Range.clip(pulse, 0, 1);
                light.setPosition(pulse);
                break;

            case RAINBOW:
                // map time into 0-1 for smooth color cycling
                double rainbow = (t % 2000) / 2000.0;
                light.setPosition(rainbow);
                break;

            case FLASH:
                // toggle between two colors
                if ((t / 500) % 2 == 1)
                    light.setPosition(color);
                else
                    light.setPosition(0);
                break;

            case WAKE:
                // flash awake, then go solid (used on init for style effect)
                if (t < 400)
                    light.setPosition(color);
                else if (t < 800)
                    light.setPosition(0);
                else if (t < 1200)
                    light.setPosition(color);
                else if (t < 1600)
                    light.setPosition(0);
                else
                {
                    light.setPosition(color);
                    setMode(LEDMode.SOLID);
                }
                break;

            case PULSE_WAKE:
                // slow breathing effect until it hits target color
                double pulseWake = 0.5 + 0.22 * Math.sin(t / 300.0);
                light.setPosition(pulseWake);
                if (pulseWake > color - 0.02 && pulseWake < color + 0.02 && t > 1500)
                {
                    light.setPosition(color);
                    setMode(LEDMode.SOLID);
                }
                break;
        }
    }
}
