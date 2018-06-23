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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

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
    private DcMotor strafeDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor armMotor = null;
    private Servo armLeftServo = null;
    private Servo armRightServo = null;
    private Servo armMainServo = null;
    private ColorSensor middleBar = null;
    private  ColorSensor bottomBar = null;
    //private TouchSensor touchSensor = null;

    //Fields for setting power
    private double MOTOR_MAX = 1.0;
    private double MOTOR_OFF = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing ...");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
       // strafeDrive = hardwareMap.get(DcMotor.class, "strafe_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armRightServo = hardwareMap.get(Servo.class, "arm_right_servo");
        armLeftServo = hardwareMap.get(Servo.class, "arm_left_servo");
        armMainServo = hardwareMap.get(Servo.class, "arm_main_servo");
        //touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
//        middleBar   = hardwareMap.get(ColorSensor.class, "middle");
//        bottomBar   = hardwareMap.get(ColorSensor.class, "bottom");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMainServo.scaleRange( -0.5, 1.0);
        armMainServo.setPosition(0.0);
        armLeftServo.setPosition(0);
        armRightServo.setPosition(0);

        // Tell the driver that initialization is complete.
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

        //region Wheels

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double horizontalPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive =  gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn  =  gamepad1.right_stick_x;

        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        horizontalPower = Range.clip( drive + strafe, -1.0,1.0);

        // Send calculated power to wheels

        // Normal Speed
        leftDrive.setPower(leftPower * 0.4);
        rightDrive.setPower(rightPower * 0.4);
//        strafeDrive.setPower(horizontalPower * 0.4);

        // Turbo Speed
        if (gamepad1.left_stick_button){
            leftDrive.setPower(leftPower * 0.9);
            rightDrive.setPower(rightPower * 0.9);
  //          strafeDrive.setPower(horizontalPower * 0.9);
        }

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

        if(gamepad1.a && gamepad1.dpad_up){
            armMotor.setPower(MOTOR_MAX);
        }
        else if(gamepad1.a && gamepad1.dpad_down){
            armMotor.setPower(-MOTOR_MAX);
        }
        else{
            armMotor.setPower(MOTOR_OFF);
        }

        //endregion

        //region gripServoMotors

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
        else if(gamepad1.a && (!gamepad1.dpad_up || !gamepad1.dpad_down)) {
            automatedServoPlacement();
        }

        armRightServo.setPosition(rightServoPosition);
        armLeftServo.setPosition(leftServoPosition);

        //endregion

        //region mainServoMotor
        double servoSpeed = gamepad1.right_stick_y;

        double mainServoPosition = armMainServo.getPosition();
        armMainServo.setPosition(servoContinuousMotion(servoSpeed, mainServoPosition));


    /* Petr: We can now control the main servo using the right joystick */
//        if(gamepad1.right_bumper){
//            mainServoPosition += 0.1;
//        }
//        else if(gamepad1.left_bumper){
//            mainServoPosition -= 0.1;
//        }
//        else {
//
//        }

        //endregion

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), horizontal (%.2f)", leftPower, rightPower, horizontalPower);
        telemetry.addData("Servos", "left (%.2f), right (%.2f), main_arm (%.4f)", leftServoPosition, rightServoPosition, mainServoPosition);
        //telemetry.addData("cageMotor", "Position: " + cageMotor.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void automatedServoPlacement (){
        /* HOW IT WILL WORK
        * 1. if sensors bottom and middle are active then place block on top
         * 2. if bottom sensor is active but middle is not then place block in the middle slot
         * 3. If both sensors are inactive then put on the bottom slot
        * */
    }

    private double servoContinuousMotion (double servoSpeed, double servoMovingPosition) {

        if (servoSpeed > 0.05){
            servoMovingPosition = servoSpeed * 0.006;
        } else if (-0.05 > servoSpeed){
            servoMovingPosition = servoSpeed * 0.006;
        }
        return servoMovingPosition;
    }

    private void flipAndDropOff(){
        // Stage 1 move hand into compartment
        armMainServo.setPosition(-0.5);

        // Stage 2 open grip
        if (armMainServo.getPosition() == -0.5) {
            double leftServoPosition = armLeftServo.getPosition();
            double rightServoPosition = armRightServo.getPosition();
            leftServoPosition -= 0.1;
            rightServoPosition += 0.1;
            armRightServo.setPosition(rightServoPosition);
            armLeftServo.setPosition(leftServoPosition);
        }

        //Stage 3 return hand
        armMainServo.setPosition(0.0);
    }

}
