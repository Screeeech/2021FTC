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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
@Autonomous(name = "Deliver 2 SkyStones - Blue", group = "Concept")
//@Disabled
public class JaguarFTC1Autonomous_2Stones_Blue extends LinearOpMode {

    // Fine tune data
    private int clawForwardTime = 600; // milliseconds. The time to move slide forward, which control the claw forward distance.
    private int clawBackwardTime = 800;

    private double backupDistance = 5.0; // The distance to move the robot backward after grabbing the stone
    private double distanceToOtherSide = 65; // The distance to move to the other side of the field during autonomous after grabbing the stone
    private double driveBackConstant = 24.0;
    private double distanceToPark = 32.0;
    static private final double STOP_DISTANCE = 12.0;


    static private final int STONE_WIDTH = 8;
    private boolean secondSkystoneAgainstWall = false;
    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robotInit(hardwareMap, telemetry);
        //readFineTuneData();


        /** Wait for the game to begin */
        waitForStart();

        // Phase 2: Deliver 1st Skystone
        // Assume the robot is perfectly aligned with the center of the stone during setup,
        // then the skystone must be offset either 8, -8, or 0 inches from the center of robot.

        //      Step 2.1: Move robot close to skystone
        // Method 1: Use motor encoder and light sensor to control the forwarding distance
        //driver.driveForward(driver.movingSpeed, 22, driver.FACING_NORTH);
        //driver.lightSensorDriveToObject(driver.FACING_NORTH);
        // Method 2: Use distance sensor to control the forwarding distance
		driver.distanceSensorDriveToObject(STOP_DISTANCE);
		// Following three lines are Skystone calibration code. Need to uncomment during calibration and comment out rest code.
        //telemetry.addData("Stone hue: ", getHueFromColorSensor(robot.sensorColor));
        //telemetry.update();
        //sleep(20000);
        alignWithSkystone();

        //      Step 2.2: Claw grabs stone
        clawPickupStone(clawForwardTime);

        //      Step 2.3: Moving back by gyro driver backward
        driver.driveBackward(driver.movingSpeed, backupDistance, driver.FACING_NORTH);
        sleep(50);

        //      Step 2.4: Moving to other side of the field
        driver.gyroTurn(driver.FACING_WEST);
        driver.driveForward(driver.movingSpeed, distanceToOtherSide, driver.FACING_WEST); // Moving 65 inches to make sure the one at the right can be delivered

        //      Step 2.5: Drop the stone
        clawDropStone();
        driver.turnClawHorizontal();
        driver.moveClawBackward(clawBackwardTime);
        driver.gyroTurn(driver.FACING_NORTH);

        //      Step 2.6: Drive back
        if (secondSkystoneAgainstWall){
            driveBackConstant = STONE_WIDTH; // Since the other Skystone is the first one by the wall, we just pick up a Stone.
        }
        double distanceToSecondSkystone = distanceToOtherSide + driveBackConstant;
        driver.driveSidewayRight(driver.movingSpeed, distanceToSecondSkystone, driver.FACING_NORTH);
        sleep(50);





        // Phase 3: Deliver 2nd Skystone
        //      Step 3.1: Drive close to the 2nd Skystone or a regular stone
        driver.lightSensorDriveToObject(driver.FACING_NORTH);

        //      Step 3.2: repeat the same steps as the first skystone
        clawPickupStone(clawForwardTime);

        //      Step 3.3: Moving back by gyro driver backward
        driver.driveBackward(driver.movingSpeed, backupDistance,driver.FACING_NORTH);
        sleep(50);

        //      Step 3.4: Moving to other side of the field
        driver.gyroTurn(driver.FACING_WEST);
        driver.driveForward(driver.movingSpeed, distanceToSecondSkystone,driver.FACING_WEST);

        //      Step 3.5: Drop the stone
        clawDropStone();
        driver.turnClawHorizontal();
        driver.moveClawBackward(clawBackwardTime);
        driver.gyroTurn(driver.FACING_NORTH);





        // Phase 4: parking
        // Method 1: Park using motor encoder to control stop
        driver.driveSidewayRight(driver.movingSpeed, distanceToPark, driver.FACING_NORTH);
        // Method 2: Park using color sensor to control stop
        //park();
    }




    private void park () {
        driver.driveSidewayRight(.6, JaguarFTC1Driver.FACING_NORTH);
        int hue = robot.parkingColorSensor.argb();

        //while (opModeIsActive()) {
        while(hue < JaguarFTC1Driver.HUE_FLOOR_THRESHOLD) {
            //telemetry.addData("hue", hue);
            //telemetry.update();
            hue = robot.parkingColorSensor.argb();
        }

        driver.stop();
    }

    private void alignWithSkystone () {
        // ASSUME: robot is aligned with the first stone (away from wall)
        float hue = getHueFromColorSensor(robot.sensorColor);
        //telemetry.addData("Stone hue: ", hue);
        //telemetry.update();
        int count = 0;
        // ASSUME: the first skystone must be among the first three stones
        while (hue < JaguarFTC1Driver.HUE_THRESHOLD && count < 2) {
            // check the next stone
            driver.driveSidewayRight(JaguarFTC1Driver.movingSpeed, STONE_WIDTH+2, JaguarFTC1Driver.FACING_NORTH);
            hue = getHueFromColorSensor(robot.sensorColor);
            //telemetry.addData("Stone hue: ", hue);
            count++;
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
        driver.openClaw();
        sleep(300);
    }








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
    }
}
