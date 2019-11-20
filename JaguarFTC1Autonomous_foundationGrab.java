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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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
@Autonomous(name = "JaguarFTC1AutonomousFoundationGrab", group = "Concept")
//@Disabled
public class JaguarFTC1Autonomous_foundationGrab extends LinearOpMode {

    // for now we only define two directions as we only need to move left or right to find a skystone
    // later we may need to have four directions (i.e., N/S/W/E).
    private static final int DIRECTION_LEFT = 0;
    private static final int DIRECTION_RIGHT = 1;

    // the following constants need to be adjusted accordingly

    private double forwardSpeed = 0.2;
    private double adjustSpeed = 0.05;
    private double slowdownSpeed = 0.1;
    private double movingSpeed = 0.7; // normal moving speed during autonomous
    private int clawForwardTime = 600; // milliseconds. The time to move slide forward, which control the claw forward distance.

    // Fine tune data
    double backupDistance = 15.0; // The distance to move the robot backward after grabbing the stone
    double distanceToOtherSide = 48.0; // The distance to move to the other side of the field during autonomous after grabbing the stone
    double distanceToPark = 19.0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private boolean firstStone = true;
    private double driveBackConstant = 20.0;

    private final int GRAB = 1;
    private final int RELEASE = -1;

    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robotInit(hardwareMap, telemetry);
        /** Wait for the game to begin */
        waitForStart();
        driver.driveForward(movingSpeed,42);
        driver.driveForward(movingSpeed/3,5);
        sleep(500);
        driver.driveSidewayLeft(movingSpeed,22);
        sleep(1000);
        driver.grabFoundation(RELEASE);
        sleep(500);
        driver.driveBackward(movingSpeed,49);
        sleep(1000);
        driver.grabFoundation(GRAB);
        sleep(500);
        driver.driveSidewayRight(movingSpeed,80);
    }

    /**
     * This method is used to adjust the robot's position to the (approximate) center of the skystone.
     * It is supposed to be called after the robot detects a skystone and stops.
     */

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
    }
    /*
    private void grabFoundation(Servo){
        robot.grabberUp = true;
        robot.foudationGrabberServo.setDirection(Servo.Direction.FORWARD);
        robot.foudationGrabberServo.setPosition(SERVO_SET_POSITION);
    }
*/
}
