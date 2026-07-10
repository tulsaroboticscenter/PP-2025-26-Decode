package org.firstinspires.ftc.teamcode.Training;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Variables",group="Test")
public class VariableExamples  extends OpMode {

    // comment styles: // per line or the /* */ pair

 /*
    private class variables are only visible within the class but are visible to all
    the methods of the class like init and loop.
 */
    private int ticksPerRevolution;  // defaults to 0
    private double motorPower; // defaults to 0.0
    private boolean intakeDown; // defaults to false
    public int totalScore = 0;
    private boolean clawOpened, motorsStopped, autoDone; // multiple variables on a line

    private String teamName = "Project Peacock"; // NOT a data type actually. strings are classes so they are capitalized
private int teamNumber;

    @Override
    public void init() {

        ticksPerRevolution = 1200; //  isible within the init method
        motorPower = 0.0;
        intakeDown = false;
        totalScore = 0;

        teamNumber = 10355; // only visible in the init method
        telemetry.addData("Team",teamNumber);
        telemetry.addData("Team Name",teamName);
       }

    @Override
    public void loop() {
        motorPower = .75; // visible in the loop method as well
        telemetry.addData("Team",teamNumber); // out of scope variable
    }
}
