/* Copyright (c) 2019 FIRST. All rights reserved.
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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BlueAuton", group = "Concept")
//@Disabled
public class BlueAuton extends LinearOpMode {

    // the following constants need to be adjusted accordingly
    static private final double HUE_FLOOR_THRESHOLD = 200000000;
    private static final double STOP_DISTANCE = 10;
    private static final double DISTANCE_ALIGN_ROBOT_FOUNDATION = 13;
    private static final double DISTANCE_PARK = 50.75;

    private static final double STOP_DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION = 20.0F;

    private static final double DEGREE_FOUNDATION_TURN = 20;

    // CHECK if these two constants defined correctly

    private double slowdownSpeed = 0.1;
    private double movingSpeed = 0.7; // normal moving speed during autonomousy7


    Bot14787         robot   = new Bot14787();
    GyroFuncDriver    driver = new GyroFuncDriver(robot,this);

    @Override
    public void runOpMode() throws InterruptedException {

        // initialization
        robot.init(hardwareMap, telemetry);
        RobotLog.d(String.format("FTC1LOG - Move Foundation - Red"));

        /** Wait for the game to begin */
        waitForStart();


        // Step 1: Drive forward to the red line
        driver.driveForward(driver.movingSpeed, driver.DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION, driver.FACING_NORTH);
        // Step 2: Drive(strafe right) in order to position the robot to shoot in the high goal
        driver.driveSidewayRight(driver.movingSpeed, 9, driver.FACING_NORTH);
        // Step 3: Shoot preloaded rings in the high goal
        driver.shootRing(35);
        driver.flickRing(20);
        // Step 4: Park over the red line
        driver.driveForward(driver.movingSpeed, 4, driver.FACING_NORTH);

        sleep(100);


        telemetry.addData("Mission Accomplished!!", "");
        telemetry.update ();

    }


}