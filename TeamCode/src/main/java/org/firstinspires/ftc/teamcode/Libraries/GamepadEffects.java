package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.Gamepad.LedEffect;

public class GamepadEffects {

    public GamepadEffects(){}

    public LedEffect wakeBlue = new LedEffect.Builder()
            .addStep(0,0,1,100)
            .addStep(0,0,0,100)
            .addStep(0,0,1,100)
            .addStep(0,0,0,100)
            .addStep(0,0,1,100)
            .addStep(0,0,0,100)
            .addStep(0,0,1,3000)
            .build();

    public LedEffect wakeRed = new LedEffect.Builder()
            .addStep(1,0,0,100)
            .addStep(0,0,0,100)
            .addStep(1,0,0,100)
            .addStep(0,0,0,100)
            .addStep(1,0,0,100)
            .addStep(0,0,0,100)
            .addStep(1,0,0,3000)
            .build();
}
