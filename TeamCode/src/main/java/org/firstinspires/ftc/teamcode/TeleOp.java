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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

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
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    //private DcMotor armMotor = null;
    private Servo armLeftServo = null;
    private Servo armRightServo = null;
    private Servo armMainServo = null;
    //private TouchSensor touchSensor = null;

    //Fields for setting power
    private double MOTOR_MAX = 1.0;
    private double MOTOR_OFF = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        //armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armRightServo = hardwareMap.get(Servo.class, "arm_right_servo");
        armLeftServo = hardwareMap.get(Servo.class, "arm_left_servo");
        //touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        armMainServo = hardwareMap.get(Servo.class, "arm_main_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        armMainServo.setPosition(0);
        armLeftServo.setPosition(0);
        armRightServo.setPosition(0);
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

        //region Wheels

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        //endregion

        //region liftMotor

        double liftHeight;
        //double liftPower;

        if(gamepad1.dpad_up){
            liftMotor.setPower(MOTOR_MAX);
            liftHeight = liftMotor.getCurrentPosition();
        }
        else if(gamepad1.dpad_down){
            liftMotor.setPower(-MOTOR_MAX);
            liftHeight = liftMotor.getCurrentPosition();
        }
        else {
            liftMotor.setPower(MOTOR_OFF);
        }
        //liftMotor.setPower(liftPower);

        //endregion

        //region armMotor --> not in use

        //if(gamepad1.a && gamepad1.dpad_up){
            //armMotor.setPower(MOTOR_MAX);
        //}
        //else if(gamepad1.a && gamepad1.dpad_down){
            //armMotor.setPower(-MOTOR_MAX);
        //}
       // else{
            //armMotor.setPower(MOTOR_OFF);
        //}

        //endregion

        //region armServoMotors

        double leftServoPosition = armLeftServo.getPosition();
        double rightServoPosition = armRightServo.getPosition();

        if(gamepad1.x){
            leftServoPosition += 0.1;
            rightServoPosition -= 0.1;
        }
        else if(gamepad1.b){
            leftServoPosition -= 0.1;
            rightServoPosition += 0.1;
        }
        else {

        }
        armRightServo.setPosition(rightServoPosition);
        armLeftServo.setPosition(leftServoPosition);

        //endregion

        //region mainServoMotor

        double mainServoPosition = armMainServo.getPosition();

        if(gamepad1.right_bumper){
            mainServoPosition += 0.1;
        }
        else if(gamepad1.left_bumper){
            mainServoPosition -= 0.1;
        }
        else {

        }
        armMainServo.setPosition(mainServoPosition);

        //endregion

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("liftMotor", "Height: " + liftMotor.getCurrentPosition());
        telemetry.addData("armLeftServo", "position: " + leftServoPosition);
        telemetry.addData("armRightServo", "position: " + rightServoPosition);
        telemetry.addData("armMainServo", "position" + mainServoPosition);
        //telemetry.addData("cageMotor", "Position: " + cageMotor.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
