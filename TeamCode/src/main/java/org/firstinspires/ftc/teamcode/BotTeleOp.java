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
    private boolean intakePressed = false;
    private boolean shooterPressed = false;

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
                robot.leftIntake.setPower(0.8);
                robot.rightIntake.setPower(0.8);
                intakePressed = true;
            }
            if(gamepad1.a && intakePressed == true){
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
                intakePressed = false;
            }
            if(gamepad1.y && shooterPressed == false){
                robot.leftShooter.setPower(0.75);
                robot.rightShooter.setPower(0.75);
                shooterPressed = true;
            }
            if(gamepad1.y && shooterPressed == true){
                robot.leftShooter.setPower(0);
                robot.rightShooter.setPower(0);
                shooterPressed = false;
            }


            // MECANUM WHEEL DRIVE
            double forward = -gamepad1.left_stick_y;
            double strafe  =  -gamepad1.left_stick_x;
            double turn   =  -gamepad1.right_stick_x;

            robot.leftFront.setPower(Range.clip(forward + strafe + turn, -0.9, 0.9));
            robot.rightFront.setPower(Range.clip(forward - strafe - turn, -0.9, 0.9)) ;
            robot.leftBack.setPower(Range.clip(forward + strafe - turn, -0.9, 0.9));
            robot.rightBack.setPower(Range.clip(forward - strafe + turn, -0.9, 0.9));

            //

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}