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

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import android.graphics.Color;
import android.os.Environment;

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
@Autonomous(name = "Deliver SkyStone and Move Foundation - Blue", group = "Concept")
//@Disabled
public class JaguarFTC1Autonomous_StoneFoundation_Blue extends LinearOpMode {

    // Fine tune data
    private int clawForwardTime = 600; // milliseconds. The time to move slide forward, which control the claw forward distance.
    private int clawBackwardTime = 800;

    private double backupDistance = 5.0; // The distance to move the robot backward after grabbing the stone
    private double distanceToFoundation = 79; // The distance from the first stone to the center of foundation
    private double driveBackConstant = 24.0;
    private double distanceToPark = 20.75;


    static private final int STONE_WIDTH = 8;
    private boolean secondSkystoneAgainstWall = false;
    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robotInit(hardwareMap, telemetry);
        RobotLog.d(String.format("FTC1LOG - Deliver SkyStone and Move Foundation - Blue"));
        //readFineTuneData();


        /** Wait for the game to begin */
        waitForStart();

        // Phase 2: Deliver 1st Skystone
        // Assume the robot is perfectly aligned with the center of the stone during setup,
        // then the skystone must be offset either 8, -8, or 0 inches from the center of robot.

        //      Step 2.1: Move robot close to skystone
		/* Uncomment out following lines if want to stop without Braking.
        DcMotor.ZeroPowerBehavior originalBehavior = robot.baseFrontLeftMotor.getZeroPowerBehavior();
        robot.baseFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.baseFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.baseBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.baseBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		*/
        driver.driveForward(driver.movingSpeed, 22, driver.FACING_NORTH);
        /*
        robot.baseFrontLeftMotor.setZeroPowerBehavior(originalBehavior);
        robot.baseFrontRightMotor.setZeroPowerBehavior(originalBehavior);
        robot.baseBackLeftMotor.setZeroPowerBehavior(originalBehavior);
        robot.baseBackRightMotor.setZeroPowerBehavior(originalBehavior);
        */
        // Method 1: Use motor encoder and light sensor to control the forwarding distance
        //driver.lightSensorDriveToObject(driver.FACING_NORTH);
        // Method 2: Use distance sensor to control the forwarding distance
        double distance = driver.distanceSensorDriveToObject(JaguarFTC1Driver.slowdownSpeed, JaguarFTC1Driver.STOP_DISTANCE);
        RobotLog.d(String.format("FTC1LOG - To Stone: Poweroff Encoder %d, Distance %4.1f", robot.baseBackLeftMotor.getCurrentPosition(), distance));
        sleep(1000);
        alignWithSkystone();

        //      Step 2.2: Claw grabs stone
        clawPickupStone(clawForwardTime);

        //      Step 2.3: Moving back by gyro driver backward
        driver.driveBackward(driver.movingSpeed, backupDistance, driver.FACING_NORTH);
        sleep(50);

        //      Step 2.4: Moving to other side of the field
        driver.gyroTurn(driver.FACING_WEST);
        driver.driveForward(driver.movingSpeed, distanceToFoundation+2, driver.FACING_WEST);

        //      Step 2.5: Detect the foundation and drop the stone
        driver.gyroTurn(driver.FACING_NORTH);
        dropStoneOnFoundation();



        // Phase 3: Move foundation
        //      Step 3.1: Grab foundation
        driver.grabFoundation(driver.GRAB);
        sleep(500); // give time to ensure the foundation is grabbed

        //      Step 3.2: Move foundation
        // Method 1: Move foundation by setting target position
        //driver.driveBackward(driver.movingSpeed, driver.DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION+2, driver.FACING_NORTH);
        //sleep(100);
        // Method 2: Drive back to the wall by setting target position and checking if the wheels stuck
        //driver.driveBackwardToWall(driver.movingSpeed, driver.DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION+2, JaguarFTC1Driver.FACING_NORTH);
        // Method 3: Drive back to the wall by checking if touch sensors have been pressed
        driver.driveBackwardToWall(driver.movingSpeed, JaguarFTC1Driver.FACING_NORTH);
        //      Step 3.3: Release foundation
        driver.grabFoundation(driver.RELEASE);
        sleep(500); // wait to ensure foundation is release



        // Phase 4: parking
        //      Step 4.1: Move robot out of foundation
        driver.driveSidewayRight(driver.movingSpeed, 33, driver.FACING_NORTH);
        sleep(100);
        driver.driveForward(driver.movingSpeed, 24, driver.FACING_NORTH);

        //      Step 4.2: Park
        driver.driveSidewayRight(driver.movingSpeed, distanceToPark, driver.FACING_NORTH);
    }




    private void alignWithSkystone () {
        // ASSUME: robot is aligned with the first stone (away from wall)
        float hue = getHueFromColorSensor(robot.sensorColor);
        double distance = robot.sensorRange.getDistance(DistanceUnit.INCH);
        RobotLog.d(String.format("FTC1LOG - Stone Detect: Distance: %4.1f, HUE 1: %5.1f", distance, hue));
        //telemetry.addData("Stone hue: ", hue);
        //telemetry.update();
        int count = 0;
        // ASSUME: the first skystone must be among the first three stones
        while (hue < JaguarFTC1Driver.HUE_THRESHOLD && count < 2) {
            // check the next stone
            driver.driveSidewayRight(JaguarFTC1Driver.movingSpeed, STONE_WIDTH+2, JaguarFTC1Driver.FACING_NORTH);
            hue = getHueFromColorSensor(robot.sensorColor);
            distance = robot.sensorRange.getDistance(DistanceUnit.INCH);
            RobotLog.d(String.format("FTC1LOG - Stone Detect: Distance: %4.1f, HUE %d: %5.1f", distance, count+2, hue));
            //telemetry.addData("Stone hue: ", hue);
            count++;
            distanceToFoundation += 8;
        }

        if (hue < JaguarFTC1Driver.HUE_THRESHOLD) {
            telemetry.addData("CANNOT FIND THE FIRST SKYSTONE", "!!!");
        }

        if (count == 2) {
            telemetry.addData("2nd Skystone is by the wall", "!!!"); // the 2nd stone is the first one next to wall
            secondSkystoneAgainstWall = true;
        }

        telemetry.update();
    }

    public float getHueFromColorSensor(ColorSensor sensorColor) {

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        float hsvValues [] = {0F, 0F, 0F};

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        float hsvValuesSum = 0;
        for (int i = 0; i < 5; i ++) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            hsvValuesSum += hsvValues[0];
        }

        return hsvValuesSum / 5;
    }




    private void clawPickupStone(int clawForwardTime) {
        driver.moveClawForward(clawForwardTime);

        // Turn claw head 90 degree
        driver.turnClawVertical();

        // Grab Skystone
        driver.closeClaw();

        // Move claw close to robot
        driver.moveClawBackward(clawForwardTime/3);
    }

    public void clawDropStone() {

        // Open claw
        driver.moveClawForward(200);
        driver.openClaw();
        sleep(300);
    }

    private void dropStoneOnFoundation() {
        driver.liftUp(driver.LIFT_POWER,30);

        //driver.driveForward(driver.movingSpeed, 4, driver.FACING_NORTH);
        // Method 1: Move to the foundation by using light sensor
        //driver.lightSensorDriveToObject(driver.lightSensorSlowdownSpeed, driver.STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES, driver.FACING_NORTH);
        // Method 2: Move to the foundation by using both light sensor and distance sensor
        driver.driveToFoundation(driver.lightSensorSlowdownSpeed, driver.FACING_NORTH);

        // Step 10: Open claw to drop the stone
        clawDropStone();
        //driver.driveBackward(driver.movingSpeed,2,driver.FACING_NORTH);

        driver.turnClawHorizontal();
        driver.moveClawBackward(clawBackwardTime);

        driver.liftDown(-0.4);
    }







    /*
    private void readFineTuneData() {
        String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Download";
        JsonReader opmodeCfgs = new JsonReader(path+"/JaguarAutoConfig.json");
        //telemetry.addData("", "Config file path: " + path);
        try {
            JSONArray cfgArray = opmodeCfgs.jsonRoot.getJSONArray("JaguarMoves");
            for(int i=0; i<cfgArray.length(); i++) {
                JSONObject move = cfgArray.getJSONObject(i);
                backupDistance = Double.parseDouble(move.getString("backupDistance")); // Init to 15.0
                distanceToOtherSide = Double.parseDouble(move.getString("distanceToOtherSide")); // Init to 48
                distanceToPark = Double.parseDouble(move.getString("distanceToPark")); // Init to 19
            }
        } catch (JSONException e) {
            e.printStackTrace();}
    }*/
}
