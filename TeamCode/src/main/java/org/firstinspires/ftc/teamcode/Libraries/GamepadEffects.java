package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.Gamepad.LedEffect;

public class GamepadEffects {

    public GamepadEffects(){}

    public LedEffect wakeBlue = new LedEffect.Builder()
            .addStep(0,0,1,400)
            .addStep(0,0,0,400)
            .addStep(0,0,1,400)
            .addStep(0,0,0,400)
            .addStep(0,0,1,3000)
            .build();

    public LedEffect wakeRed = new LedEffect.Builder()
            .addStep(1,0,0,400)
            .addStep(0,0,0,400)
            .addStep(1,0,0,400)
            .addStep(0,0,0,400)
            .addStep(1,0,0,3000)
            .build();

    public LedEffect flashRed = new LedEffect.Builder()
            .addStep(1,0,0, 250)
            .addStep(0,0,0,250)
            .setRepeating(true)
            .build();

    public LedEffect flashYellow = new LedEffect.Builder()
            .addStep(1,1,0, 250)
            .addStep(0,0,0,250)
            .setRepeating(true)
            .build();

    public LedEffect flashGreen = new LedEffect.Builder()
            .addStep(0,1,0, 250)
            .addStep(0,0,0,250)
            .setRepeating(true)
            .build();

    public LedEffect flashBlue = new LedEffect.Builder()
            .addStep(0,0,1, 250)
            .addStep(0,0,0,250)
            .setRepeating(true)
            .build();
}
