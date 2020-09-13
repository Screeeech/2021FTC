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
@Autonomous(name = "Move Foundation - Blue", group = "Concept")
//@Disabled
public class JaguarFTC1AutonomousFoundationGrab_Blue extends LinearOpMode {

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


    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);

    @Override
    public void runOpMode() throws InterruptedException {

        // initialization
        robot.robotInit(hardwareMap, telemetry);
        RobotLog.d(String.format("FTC1LOG - Move Foundation - Blue"));

        /** Wait for the game to begin */
        waitForStart();


        // Step 1: Move sideway left to align robot and foundation
        driver.driveSidewayLeft(driver.movingSpeed, DISTANCE_ALIGN_ROBOT_FOUNDATION, driver.FACING_NORTH);

        // Step 2: Drive forward to foundation
        driver.driveForward(driver.movingSpeed, driver.DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION-driver.DISTANCE_SLOWDOWN, driver.FACING_NORTH);
        // Method 1: Move to the foundation by using light sensor
        //driver.lightSensorDriveToObject(driver.lightSensorSlowdownSpeed, driver.STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES, driver.FACING_NORTH);
        // Method 2: Move to the foundation by using both light sensor and distance sensor
        driver.driveToFoundation(driver.lightSensorSlowdownSpeed, driver.FACING_NORTH);
        sleep(100);

        // Step 3: Grab foundation
        driver.grabFoundation(driver.GRAB);
        sleep(500); // give time to ensure the foundation is grabbed

        // Step 4: Drive backward to the wall
        // Method 1: Move foundation by setting target position
        //driver.driveBackward(movingSpeed, driver.DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION+2, driver.FACING_NORTH);
        // Method 2: Drive back to the wall by setting target position and checking if the wheels stuck
        //driver.driveBackwardToWall(movingSpeed, driver.DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION+2, JaguarFTC1Driver.FACING_NORTH);
        // Method 3: Drive back to the wall by checking if touch sensors have been pressed
        driver.driveBackwardToWall(movingSpeed, JaguarFTC1Driver.FACING_NORTH);


        // Step 5: Release foundation
        driver.grabFoundation(driver.RELEASE);
        sleep(500); // wait to ensure foundation is release

        // Step 6. Drive right to park
        driver.driveSidewayRight(movingSpeed, DISTANCE_PARK, driver.FACING_NORTH);

        telemetry.addData("Mission Accomplished!!", "");
        telemetry.update ();

    }


}
