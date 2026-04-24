package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Classes.RGBLightController;
import org.firstinspires.ftc.teamcode.Robot.HardwareManager;

@TeleOp(group = "Robot", name = "Recalibrate")
public class RecalibratePinpoint extends OpMode {

    HardwareManager hw = null;

    @Override
    public void init()
    {
        hw = new HardwareManager(hardwareMap);

        hw.pinpoint.init(hardwareMap, true);
        hw.lights.init(hardwareMap, true);
        hw.lights.setLightMode(RGBLightController.LEDMode.SOLID);
    }

    @Override
    public void init_loop()
    {
        hw.lights.setLightColor(hw.pinpoint.isReady() ? RGBLightController.GREEN : RGBLightController.RED);
        hw.lights.update();
        hw.pinpoint.update();
    }

    @Override
    public void start()
    {
        hw.pinpoint.pinpoint.recalibrateIMU();
    }

    @Override
    public void loop()
    {
        hw.lights.setLightColor(hw.pinpoint.isReady() ? RGBLightController.GREEN : RGBLightController.RED);
        hw.lights.update();
        hw.pinpoint.update();
    }

}
