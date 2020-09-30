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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// I included this mainly as a way to show how Bot and Driver can be used. We will need to overhaul all of this

@TeleOp(name="JaguarFTC1UserControl", group="Linear Opmode")
//@Disabled
public class JaguarFTC1UserControl extends LinearOpMode {

    private JaguarFTC1Bot         robot   = new JaguarFTC1Bot();

    double NORMAL_DRIVE_POWER_FACTOR = 1.0; // The power factor for normal drive
    double SLOW_DRIVE_POWER_FACTOR = 0.5; // The power factor for slow drive
    double LIFT_POWER = 1.0; // The power for Arm forward/backward
    double LIFT_SLOW_POWER = 0.7;
    double LIFT_DOWN_POWER = -0.2;
    double MIN_HOLDING_ENCODER_VAL = 15;
    double LIFT_HOLDING_POWER = 0.2;
    //int MAX_LIFT_ENCODER_VAL = 93; // Test result when the arm is stretched to the longest. Left: 1750, Right: 2100

    static final double TOLERANCE = 0.15;
    static final double VEX_MOTOR_RUN = 0.7;     // The servo position to open latch
    static final double VEX_MOTOR_STOP = 0.0;
    static final double SERVO_SET_POSITION = 1.0;
    static final double SERVO_INIT_POSITION = 0.0;  // The servo position to close latch

    private boolean clawButtonReleased = true;
    private boolean clawHeadButtonRelease = true;
    private boolean foundationGrabberButtonRelease = true;
    private boolean capstoneServoButtonRelease = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        int leftLiftMotor_encoderVal = 0; // Keep left Arm Stretch motor encoder reading (Arm length)
        int rightLiftMotor_encoderVal = 0; // Keep right Arm Stretch motor encoder reading (Arm length)

        robot.robotInit(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Base Control
            //                     <<<<< Left Stick Y and Right stick X >>>>>
            //
            // POV Mode uses left stick to go forward, and right stick to turn. Easier to drive straight.
            double x = -gamepad1.left_stick_y; // Forward and backward
            double y = gamepad1.left_stick_x; // Sideway left and right
            double r = gamepad1.right_stick_x; //Turn left and right

            if(Math.abs(x) > TOLERANCE || Math.abs(r) > TOLERANCE || Math.abs(y) > TOLERANCE) {
                // y to move to goldilocks height
                // right trigger to move everything slower
                manualDrive(x,y,r);
            }
            else {
                robot.baseFrontLeftMotor.setPower(0);
                robot.baseFrontRightMotor.setPower(0);
                robot.baseBackLeftMotor.setPower(0);
                robot.baseBackRightMotor.setPower(0);
            }




            // Lift Control
            //                    <<<<< DPAD Up and DPAD Down >>>>>>
            //
            // NOTE: The Left Arm Motor encoder value doesn't change at the same rate as the value of Right Arm Motor encoder
            //       even though the two motors are connected by shaft.
            //       Currently, the ticks per round of right motor are larger than those of left motor.
            //       We just use Right Arm Motor encoder value to make decision.

            leftLiftMotor_encoderVal = robot.leftLiftMotor.getCurrentPosition();
            rightLiftMotor_encoderVal = robot.rightLiftMotor.getCurrentPosition();
            telemetry.addData("Lift Encoder", "L: %7d R: %7d",
                    leftLiftMotor_encoderVal, rightLiftMotor_encoderVal);

            if (gamepad1.dpad_up) {

                if (leftLiftMotor_encoderVal>=(JaguarFTC1Bot.MAX_LIFT_ENCODER_VAL*0.8)) { // Slow down if close to the longest
                    robot.leftLiftMotor.setPower(LIFT_SLOW_POWER);
                    robot.rightLiftMotor.setPower(LIFT_SLOW_POWER);
                }
               else {
                    robot.leftLiftMotor.setPower(LIFT_POWER);
                    robot.rightLiftMotor.setPower(LIFT_POWER);
                }
            }
            else if (gamepad1.dpad_down) {
                robot.leftLiftMotor.setPower(LIFT_DOWN_POWER);
                robot.rightLiftMotor.setPower(LIFT_DOWN_POWER);
            }
            else {
                if (leftLiftMotor_encoderVal>MIN_HOLDING_ENCODER_VAL) {
                    robot.leftLiftMotor.setPower(LIFT_HOLDING_POWER);
                    robot.rightLiftMotor.setPower(LIFT_HOLDING_POWER);
                }
                else {
                    robot.leftLiftMotor.setPower(0);
                    robot.rightLiftMotor.setPower(0);
                }
            }





            // Claw Control
            //                      <<<<< Left Bumper >>>>>
            // Toggle the latch open and close
            //
            // If Left Bumper has been pressed and still being pressed, we consider the latch is performing toggle action now.
            // No further action is needed.

            if (gamepad1.left_bumper && clawButtonReleased) {
                clawButtonReleased = false;

                if (robot.clawOpen) {
                    robot.clawOpen = false;
                    robot.clawServo.setDirection(Servo.Direction.REVERSE);
                    robot.clawServo.setPosition(SERVO_SET_POSITION);
                }
                else {
                    robot.clawOpen = true;
                    robot.clawServo.setDirection(Servo.Direction.FORWARD);
                    robot.clawServo.setPosition(SERVO_SET_POSITION);
                }
            }
            else if (!gamepad1.left_bumper){
                clawButtonReleased = true;
            }


            // Claw Head Control
            //                      <<<<< Left Trigger >>>>>
            // Toggle the latch open and close
            //
            // If Left Bumper has been pressed and still being pressed, we consider the latch is performing toggle action now.
            // No further action is needed.

            if (gamepad1.left_trigger>0.8 && clawHeadButtonRelease) {
                clawHeadButtonRelease = false;

                if (robot.clawHeadHorizontal) {
                    robot.clawHeadHorizontal = false;
                    robot.clawheadServo.setDirection(Servo.Direction.FORWARD);
                    robot.clawheadServo.setPosition(SERVO_SET_POSITION);
                }
                else {
                    robot.clawHeadHorizontal = true;
                    robot.clawheadServo.setDirection(Servo.Direction.REVERSE);
                    robot.clawheadServo.setPosition(SERVO_SET_POSITION);
                }
            }
            else if (gamepad1.left_trigger<=0.8){
                clawHeadButtonRelease = true;
            }


            // Foundation Grabber Control
            //                      <<<<< Right Trigger >>>>>
            // Toggle the latch open and close
            //
            // If Left Bumper has been pressed and still being pressed, we consider the latch is performing toggle action now.
            // No further action is needed.

            if (gamepad1.right_trigger>0.8 && foundationGrabberButtonRelease) {
                foundationGrabberButtonRelease = false;

                if (robot.grabberUp) {
                    robot.grabberUp = false;
                    robot.foudationGrabberServo.setDirection(Servo.Direction.REVERSE);
                    robot.foudationGrabberServo.setPosition(SERVO_SET_POSITION);
                }
                else {
                    robot.grabberUp = true;
                    robot.foudationGrabberServo.setDirection(Servo.Direction.FORWARD);
                    robot.foudationGrabberServo.setPosition(SERVO_SET_POSITION);
                }
            }
            else if (gamepad1.right_trigger<=0.8){
                foundationGrabberButtonRelease = true;
            }

            //Capstone release servo
            if (gamepad1.b && capstoneServoButtonRelease) {
                capstoneServoButtonRelease = false;

                if (robot.capstoneLoaded) {
                    robot.capstoneLoaded = false;
                    robot.capstoneServo.setDirection(Servo.Direction.REVERSE);
                    robot.capstoneServo.setPosition(SERVO_SET_POSITION);
                }
                else {
                    robot.capstoneLoaded = true;
                    robot.capstoneServo.setDirection(Servo.Direction.FORWARD);
                    robot.capstoneServo.setPosition(SERVO_SET_POSITION);
                }
            }
            else if (!gamepad1.b){
                capstoneServoButtonRelease = true;
            }

            // Slide Control
            //                <<<<< DPAD Left and DPAD Right >>>>>>
            //
            // Following display is used to get the power that stops the CR servo
            //telemetry.addData("Servo Position", "%5.2f", gamepad1.right_stick_y);
            //intakeServo.setPower(0.05); // The power to stop HSR-1425CR servo

            if (gamepad1.dpad_left) {
                robot.slideServo.setDirection(Servo.Direction.FORWARD);
                robot.slideServo.setPosition(VEX_MOTOR_RUN);
            }
            else if (gamepad1.dpad_right) {
                robot.slideServo.setDirection(Servo.Direction.REVERSE);
                robot.slideServo.setPosition(VEX_MOTOR_RUN);
            }
            else {
                robot.slideServo.setPosition(VEX_MOTOR_STOP);
            }


            //sleep(CYCLE_MS);
            idle(); // For multiple threads program to give other thread a change to run.
            telemetry.update();

        }
    }


    // Mecanum wheel drive
    public void manualDrive(double x, double y, double r)
    {
        // Setup a variable for each drive wheel to save power level for telemetry
        double baseFrontLeftPower;
        double baseFrontRightPower;
        double baseBackLeftPower;
        double baseBackRightPower;

        baseFrontLeftPower = Range.clip(x + y + r, -1.0, 1.0) ;
        baseFrontRightPower = Range.clip(x - y - r, -1.0, 1.0) ;
        baseBackLeftPower = Range.clip(x - y + r, -1.0, 1.0) ;
        baseBackRightPower = Range.clip(x + y - r, -1.0, 1.0) ;

        // Send calculated power to wheels
        if (gamepad1.right_bumper){ // Slow motion
            robot.baseFrontLeftMotor.setPower(baseFrontLeftPower*SLOW_DRIVE_POWER_FACTOR);
            robot.baseFrontRightMotor.setPower(baseFrontRightPower*SLOW_DRIVE_POWER_FACTOR);
            robot.baseBackLeftMotor.setPower(baseBackLeftPower*SLOW_DRIVE_POWER_FACTOR);
            robot.baseBackRightMotor.setPower(baseBackRightPower*SLOW_DRIVE_POWER_FACTOR);
        }
        else {
            robot.baseFrontLeftMotor.setPower(baseFrontLeftPower*NORMAL_DRIVE_POWER_FACTOR);
            robot.baseFrontRightMotor.setPower(baseFrontRightPower*NORMAL_DRIVE_POWER_FACTOR);
            robot.baseBackLeftMotor.setPower(baseBackLeftPower*NORMAL_DRIVE_POWER_FACTOR);
            robot.baseBackRightMotor.setPower(baseBackRightPower*NORMAL_DRIVE_POWER_FACTOR);
        }

    }
}
