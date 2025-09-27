package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;
import org.firstinspires.ftc.teamcode.Libraries.Targeting;

@Autonomous(name = "Auto1", group = "Robot", preselectTeleOp = "Sauron")
public class auto extends LinearOpMode {

    HWProfile robot = new HWProfile();
    MechOps ops = new MechOps();
    Targeting target = new Targeting(robot, this);



    public void runOpMode() {
        waitForStart();

        if (opModeIsActive()) {



            ops.allStop();
            ops.writePose(robot.pinpoint.getPosition(), "PoseFile");
        }
    }

}
