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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Basic_Bot
{
    /* Public OpMode members. */
    protected DcMotor leftDrive = null;
    protected DcMotor rightDrive = null;
    protected DcMotor leftIntake = null;
    protected DcMotor rightIntake = null;
    protected DcMotor leftTilt  = null;
    protected DcMotor rightTilt = null;
    protected Servo   leftMoveIntake = null;
    protected Servo   rightMoveIntake = null;
    protected Servo   leftFlick     = null;
    protected Servo   rightFlick    = null;
    protected Servo   cubeStopper   = null;
    protected DcMotor solarLift = null;
    protected Servo   solarGrabber  = null;

    // Define servo positions
    protected final static double LEFT_MOVE_INTAKE_HOME= 0.2; // find home
    protected final static double RIGHT_MOVE_INTAKE_HOME= 0.2; // find home
    protected final static double INTAKE_POWER =0.5;
    protected final static double CUBE_STOPPER_HOME = 0.0; // find pos
    protected final static double CUBE_STOPPER_RELEASE = 0.0; // find pos
    protected final static double CUBE_STOPPER_STOP = 0.0; // find pos


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Basic_Bot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftIntake = hwMap.get(DcMotor.class, "left_intake");
        rightIntake = hwMap.get(DcMotor.class, "right_intake");
        leftTilt  = hwMap.get(DcMotor.class, "left_tilt");
        rightTilt = hwMap.get(DcMotor.class, "right_tilt");
        solarLift = hwMap.get(DcMotor.class, "solar_lift");


        // Define Motor Directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Positive input should drive the robot forward
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);   // Positive input should suck in the cubes :D
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        leftTilt.setPower(0);
        rightTilt.setPower(0);
        solarLift.setPower(0);

        // Define and initialize ALL installed servos.
        leftMoveIntake = hwMap.get(Servo.class, "left_intake");
        rightMoveIntake = hwMap.get(Servo.class, "right_intake");
        leftFlick   = hwMap.get(Servo.class, "left_flick");
        rightFlick = hwMap.get(Servo.class, "right_flick");
        cubeStopper = hwMap.get(Servo.class, "servo_stopper");
        solarGrabber = hwMap.get(Servo.class, "solar_grabber");

        // MAYBE init servo positions
    }
}
