package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Training.Hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.Training.Hardware.Mechanisms.ShooterHood;

@TeleOp(name="Shooter Test",group="Test")
public class ShooterHoodTest extends OpMode {

    private HardwareManager hwMgr = new HardwareManager(hardwareMap);
    @Override
    public void init() {
       hwMgr.init();


    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            hwMgr.shooterHood.moveHood(ShooterHood.HoodDirection.UP);
        } else if (gamepad1.dpad_down) {
            hwMgr.shooterHood.moveHood(ShooterHood.HoodDirection.DOWN);
        }
    }
}
