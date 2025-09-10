/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libraries.MechOps;

@TeleOp(name = "DecodeTeleOp", group = "robot")
//@Disabled
public class DecodeTeleOp extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private final static MechOps ops = new MechOps();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);

        waitForStart();
        while(opModeIsActive()) {
            ops.StrafeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            /*
             * Here we give the user control of the speed of the launcher motor without automatically
             * queuing a shot.
             */
            if (gamepad1.y) {
                robot.launcher.setVelocity(robot.LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.b) { // stop flywheel
                robot.launcher.setVelocity(robot.STOP_SPEED);
            }

            /*
             * Now we call our "Launch" function.
             */

            if (gamepad1.right_trigger > 0.5) {
                ops.launch();
                robot.feederTimer.reset();
            }

            if (robot.feederTimer.seconds() > robot.FEED_TIME_SECONDS) {
                ops.stopLaunch();
            }

            /*
             * Show the state and motor powers
             */

            telemetry.addData("launcher velocity: ", robot.launcher.getVelocity());
            telemetry.addData("feed timer (in seconds): ", robot.feederTimer.seconds());
            telemetry.update();



        }
    }
}