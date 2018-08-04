/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Git Bash stuff:
 * First go to the repository
 * Then initialize git (git init)
 * Then add (git add .)
 * Then commit (git commit -m "")-
 * Then push to master (git push origin master) --> not necessary
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Iterative Opmode")
//Disabled
public class TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Basic_Bot robo = new Basic_Bot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing ...");

        robo.init(hardwareMap);

        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    // Driving code BELOW
    double drive = gamepad1.left_stick_y;
    double turn = gamepad1.left_stick_x;
    double leftRubbPower = Range.clip(drive - turn, -1.0, 1.0);    // driving powers defined and set
    double rightRubbPower = Range.clip(drive + turn, -1.0, 1.0);


    if(gamepad1.left_stick_button){
        robo.leftDrive.setPower(leftRubbPower);
        robo.rightDrive.setPower(rightRubbPower);
    }
    // Intake code BELOW

        robo.leftIntake.setPower(robo.INTAKE_POWER);
        robo.rightIntake.setPower(robo.INTAKE_POWER);

        if(((double) gamepad1.right_trigger) != 0){
            robo.leftIntake.setPower(0);      // zucc in ;D
            robo.rightIntake.setPower(0);
        }
        if (gamepad1.b) {
            robo.leftIntake.setPower(-robo.INTAKE_POWER);    // zucc out >I
            robo.rightIntake.setPower(-robo.INTAKE_POWER);
        }

    // Moving intake arms code BELOW
        double leftZuccServoPos = robo.leftMoveIntake.getPosition();
        double rightZuccServoPos = robo.rightMoveIntake.getPosition();

        if(gamepad1.dpad_right){
            robo.leftMoveIntake.setPosition(leftZuccServoPos + 0.01);
            robo.rightMoveIntake.setPosition(rightZuccServoPos - 0.01);
        } else if(gamepad1.dpad_left){
            robo.leftMoveIntake.setPosition(leftZuccServoPos - 0.01);
            robo.rightMoveIntake.setPosition(rightZuccServoPos + 0.01);
        }
    // tilt of platform that lifts cubes
        double right_stick = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);

        if(!gamepad1.right_stick_button) {
            robo.leftTilt.setPower(right_stick);
            robo.rightTilt.setPower(-right_stick);
        }

    // solar panel lift controls
        if(gamepad1.right_stick_button){
            robo.solarLift.setPower(right_stick * 0.3);
        }
    // solar panel grab controls

    // flick release of cubes
        double leftFlickServoPos = robo.leftFlick.getPosition();
        double rightFlickServoPos = robo.rightFlick.getPosition();

        if(gamepad1.dpad_up){
            robo.leftFlick.setPosition(leftFlickServoPos + 0.01);
            robo.rightFlick.setPosition(rightFlickServoPos - 0.1);
        } else if(gamepad1.dpad_down){
            robo.leftFlick.setPosition(leftFlickServoPos - 0.1);
            robo.rightFlick.setPosition(rightFlickServoPos + 0.1);
        }
    // cube stopper
        robo.cubeStopper.getPosition();
//        if(gamepad1.right_trigger != 0){
//            robo.cubeStopper.setPosition(robo.CUBE_STOPPER_STOP);
//        } else {
//            robo.cubeStopper.setPosition(robo.CUBE_STOPPER_RELEASE);
//        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors Driving", "left (%.2f), right (%.2f)", leftRubbPower, rightRubbPower);
        telemetry.addData("Motors Intake", "left (%.2f), right (%.2f)", robo.leftIntake.getPower(), robo.rightIntake.getPower());
        telemetry.addData("Motor Lifts", "left (%.2f), right (%.2f), solar (%.2f)", robo.leftTilt.getPower(), robo.rightTilt.getPower(), robo.solarLift.getPower());
        telemetry.addData("Servos", "left_intake (%.2f), right_intake (%.2f)", leftZuccServoPos, rightZuccServoPos);
    /*
     * Code to run ONCE after the driver hits STOP
     */
    }
    @Override
    public void stop() {
    }
}
