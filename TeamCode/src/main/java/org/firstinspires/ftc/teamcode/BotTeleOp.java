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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Bot14787;


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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BotTeleOp extends LinearOpMode {

    private Bot14787 robot   = new Bot14787();

    double normalDrive = 1.0; // The power factor for normal drive
    double slowDrive = 0.5; // The power factor for slow drive
    double fullPower = 1.0; // Substitute instead of double
    double noPower = 0.0; // Substitute instead of double
    private boolean intakePressed = false;
    private boolean shooterPressed = false;
    private boolean reverseControlsPressed = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Gamepad Control for moving the robot and motors while intake donuts
            if(gamepad1.a && intakePressed == false){
                robot.backIntake.setPower(fullPower);
                robot.frontIntake.setPower(fullPower);
                intakePressed = true;
            }
            if(gamepad1.a && intakePressed == true){
                robot.backIntake.setPower(noPower);
                robot.frontIntake.setPower(noPower);
                intakePressed = false;
            }
            if(gamepad1.y && shooterPressed == false){
                robot.leftShooter.setPower(0.75);
                robot.rightShooter.setPower(0.75);
                shooterPressed = true;
            }
            if(gamepad1.y && shooterPressed == true){
                robot.leftShooter.setPower(noPower);
                robot.rightShooter.setPower(noPower);
                shooterPressed = false;
            }


            if(gamepad1.left_trigger > 0.5) {
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                if (Math.abs(forward) > 0.15 || Math.abs(strafe) > 0.15 || Math.abs(turn) > 0.15) {
                    // y to move to goldilocks height
                    // right trigger to move everything slower
                    mecanumDrive(forward, strafe, turn);
                } else {
                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.leftBack.setPower(0);
                }
            }

            else if(gamepad1.right_trigger > 0.5) {
                reverseControlsPressed = true;
                double move = -gamepad1.left_stick_y;
                double sideways = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                if (Math.abs(move) > 0.15 || Math.abs(sideways) > 0.15 || Math.abs(rotate) > 0.15) {
                    // right trigger to move everything in reverse
                    reverseControls(move, sideways, rotate);
                } else {
                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.leftBack.setPower(0);
                }

            }




            //

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        leftFrontPower = Range.clip(forward + strafe + turn, -0.9, 0.9);
        rightFrontPower = Range.clip(forward - strafe - turn, -0.9, 0.9);
        leftBackPower = Range.clip(forward + strafe - turn, -0.9, 0.9);
        rightBackPower = Range.clip(forward - strafe + turn, -0.9, 0.9);

        if (gamepad1.right_bumper){ // Slow motion
            robot.leftFront.setPower(leftFrontPower*slowDrive);
            robot.rightFront.setPower(rightFrontPower*slowDrive);
            robot.leftBack.setPower(leftBackPower*slowDrive);
            robot.rightBack.setPower(rightBackPower*slowDrive);
        }
        else {
            robot.leftFront.setPower(leftFrontPower*normalDrive);
            robot.rightFront.setPower(rightFrontPower*normalDrive);
            robot.leftBack.setPower(leftBackPower*normalDrive);
            robot.rightBack.setPower(rightBackPower*normalDrive);
        }

    }

    public void reverseControls(double move, double sideways, double rotate){
        double rightBackReverse = Range.clip(move + sideways + rotate, -0.9, 0.9);
        double leftBackReverse = Range.clip(move - sideways - rotate, -0.9, 0.9);
        double rightFrontReverse = Range.clip(move + sideways - rotate, -0.9, 0.9);
        double leftFrontReverse = Range.clip(move - sideways + rotate, -0.9, 0.9);

        if (gamepad1.right_bumper && reverseControlsPressed == true){ // Slow motion
            robot.rightBack.setPower(rightBackReverse*slowDrive);
            robot.leftBack.setPower(leftBackReverse*slowDrive);
            robot.rightFront.setPower(rightFrontReverse*slowDrive);
            robot.leftFront.setPower(leftFrontReverse*slowDrive);
        }
        else if(reverseControlsPressed == true){
            robot.rightBack.setPower(rightBackReverse*normalDrive);
            robot.leftBack.setPower(leftBackReverse*normalDrive);
            robot.rightFront.setPower(rightFrontReverse*normalDrive);
            robot.leftFront.setPower(leftFrontReverse*normalDrive);
        }
    }
}