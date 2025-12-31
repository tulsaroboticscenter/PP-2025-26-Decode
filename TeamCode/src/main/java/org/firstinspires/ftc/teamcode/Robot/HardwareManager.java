package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Lights;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Turret;

public class HardwareManager {

    public Turret turret = new Turret();
    public Intake intake = new Intake();
    public Drivetrain drivetrain = new Drivetrain();
    public Lights lights = new Lights();
    public Pinpoint pinpoint = new Pinpoint();
    public Limelight limelight = new Limelight();


    public void initTeleOp(HardwareMap hwMap)
    {
        turret.init(hwMap, true);
        intake.init(hwMap);
        lights.init(hwMap, true);
        drivetrain.init(hwMap);
        pinpoint.init(hwMap, true);
        limelight.init(hwMap, true);
    }

    public void initPedro(HardwareMap hwMap)
    {
        turret.init(hwMap, false);
        lights.init(hwMap, false);
        intake.init(hwMap);
    }

    public void updateTeleOp()
    {
        turret.update();
        lights.update();
        pinpoint.update();
        limelight.update(turret.getTurretHeadingDegrees(pinpoint.getPosition().getHeading(AngleUnit.DEGREES)));
    }

    public void updatePedro()
    {
        turret.update();
        lights.update();

    }
}
