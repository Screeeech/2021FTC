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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import static java.lang.Double.NaN;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name="Sensor: REVColorDistance", group="Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class JaguarSensorREVColorDistance extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has an IR proximity sensor which is used to calculate distance and an RGB color sensor.
     * 
     * There will be some variation in the values measured depending on whether you are using a
     * V3 color sensor versus the older V2 and V1 sensors, as the V3 is based around a different chip.
     *
     * For V1/V2, the light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * For V3, the distance sensor as configured can handle distances between 0.25" (~0.6cm) and 6" (~15cm).
     * Any target closer than 0.25" will dislay as 0.25" and any target farther than 6" will display as 6".
     *
     * Note that the distance sensor function of both chips is built around an IR proximity sensor, which is
     * sensitive to ambient light and the reflectivity of the surface against which you are measuring. If
     * very accurate distance is required you should consider calibrating the raw optical values read from the
     * chip to your exact situation.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.
     *
     */
    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);


    // when the robot moves close to the stones (prior to pickup), the robot stops at this distance from the stones.
    static public final float STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES = 7F;

    // the HUE value of a regular stone at the distance of DISTANCE_BETWEEN_ROBOT_AND_BLOCKS
    static public final float HUE_REGULAR_STONE = 20;

    // the HUE value of a Skystone at the distance of DISTANCE_BETWEEN_ROBOT_AND_BLOCKS
    static public final float HUE_SKYSTONE = 40;


    @Override
    public void runOpMode() {

        robot.robotInit(hardwareMap, telemetry);

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

        // Step 0:
        // Move to the wall
        double wallSpeed = 0.3;

        boolean pressed = robot.sensorTouch.isPressed();
        if (pressed){
            telemetry.addData("button status", "pressed");
        }else{
            telemetry.addData("button status:", "not pressed");
        }
        telemetry.update();

        robot.baseFrontLeftMotor.setPower(-1 * wallSpeed);
        robot.baseFrontRightMotor.setPower(wallSpeed);
        robot.baseBackLeftMotor.setPower(wallSpeed);
        robot.baseBackRightMotor.setPower(-1 * wallSpeed);

        while(pressed == false){
            telemetry.addData("Loop Status:", "in loop");

            if (pressed){
                telemetry.addData("button status", "pressed in loop");
            }else{
                telemetry.addData("button status:", "not pressed in loop");
            }
            telemetry.update();

            pressed = robot.sensorTouch.isPressed();
        }

        telemetry.addData("Loop Status:", "out of loop");
        telemetry.update();

        sleep(5000);

        // ASSUME that the robot is aligned with the wall and is away from the stones

        // Step 1:
        // move robot forward
        // QUESTION: Do we need to a gryoDrive method that just drives forward, i.e., without giving a specific distance?
        // gyroDrive (DRIVE_SPEED, drive_distance, 0.0);

        // Step 2:
        // keep reading the distance sensor util it is close to stones
        double correction;
        double approchSpeed=0.2;
        double distance = robot.sensorDistance.getDistance(DistanceUnit.CM);

//        telemetry.addData("Distance between robot and stones (cm)",
//                String.format(Locale.US, "%5.2f", distance));
//        telemetry.update();
//        sleep(2000);


        robot.resetAngle();

        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run at constant speed
        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.baseFrontLeftMotor.setPower(approchSpeed);
        robot.baseFrontRightMotor.setPower(approchSpeed);
        robot.baseBackLeftMotor.setPower(approchSpeed);
        robot.baseBackRightMotor.setPower(approchSpeed);

        while ((distance == DistanceSensor.distanceOutOfRange)
                || (Double.isNaN(distance))
                || (distance > STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES)) {

            // Use gyro to drive in a straight line.
            correction = driver.checkDirection();

//                opmode.telemetry.addData("1 gyro heading ", robot.lastAngles.firstAngle);
//                opmode.telemetry.addData("2 global heading ", robot.globalAngle);
//                opmode.telemetry.addData("3 correction ", correction);
//                opmode.telemetry.update();

//            robot.baseFrontLeftMotor.setPower(approchSpeed - correction);
            robot.baseFrontRightMotor.setPower(approchSpeed + correction);
//            robot.baseBackLeftMotor.setPower(approchSpeed - correction);
            robot.baseBackRightMotor.setPower(approchSpeed + correction);

            // print out info for debugging purpose
//            telemetry.addData("Distance between robot and stones (cm)",
//                    String.format(Locale.US, "%.02f", distance));
//            telemetry.update();

            // MAY need to sleep a little bit before next reading
//            sleep(50);

            distance = robot.sensorDistance.getDistance(DistanceUnit.CM);
        }

        // Step 3: Stop the robot
        // QUESTION: Do we want to have a gyroStop method or just use gyroDrive, but with specical values, e.g., DRIVE_SPEED or drive_distance set to 0?
        // gyroStop ();
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        sleep(1000);

        // Step 4:
        // move the robot sideway
        // ASSUME that the robot will go sideway when the angle is set to 90.
        // gyroDrive (DRIVE_SPEED, drive_distance, 90.0);

        // Step 5:
        // keep reading the color sensor util it detects a Skystone (i.e. a positive increase of certain threshold)
        // get the first two readings

        double scanSpeed=0.2;

        // Reset gyro
        robot.resetAngle();

        float prevHue = getHueFromColorSensor(robot.sensorColor);
        // sleep a little bit prior to next reading
        sleep(50);
        float currHue = getHueFromColorSensor(robot.sensorColor);

        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set PID mode, run to the target position
        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int moveCounts = 36*50;
        int newFrontLeftTarget = robot.baseFrontLeftMotor.getCurrentPosition() + moveCounts;
        int newFrontRightTarget = robot.baseFrontRightMotor.getCurrentPosition() - moveCounts;
        int newBackLeftTarget = robot.baseBackLeftMotor.getCurrentPosition() - moveCounts;
        int newBackRightTarget = robot.baseBackRightMotor.getCurrentPosition() + moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.baseFrontLeftMotor.setTargetPosition(newFrontLeftTarget);
        robot.baseFrontRightMotor.setTargetPosition(newFrontRightTarget);
        robot.baseBackLeftMotor.setTargetPosition(newBackLeftTarget);
        robot.baseBackRightMotor.setTargetPosition(newBackRightTarget);

        robot.baseBackRightMotor.setPower(scanSpeed);
        robot.baseFrontLeftMotor.setPower(scanSpeed);
        robot.baseBackLeftMotor.setPower(scanSpeed);
        robot.baseFrontRightMotor.setPower(scanSpeed);

        while ((currHue - prevHue) < (HUE_SKYSTONE - HUE_REGULAR_STONE)) {
            // Use gyro to drive in a straight line.
            correction = driver.checkDirection();

//                robot.baseFrontLeftMotor.setPower(speed - correction);
//                robot.baseFrontRightMotor.setPower(speed + correction);
            robot.baseBackLeftMotor.setPower(scanSpeed + correction);
            robot.baseBackRightMotor.setPower(scanSpeed + 2*correction);

            // take the next reading
            prevHue = currHue;
            // sleep a little bit prior to next reading
//            sleep(50);
            currHue = getHueFromColorSensor(robot.sensorColor);

//            telemetry.addData("Hue as detected previously", prevHue);
//            telemetry.addData("Hue as detected currently:", currHue);
//            telemetry.update();
        }

        // Step 6:
        // Here we stop the robot and then pick up the Skystone
        // gyroStop ();
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        // Stop RUN_TO_POSITION
        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public float getHueFromColorSensor (ColorSensor colorSensor) {

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        float hsvValues [] = {0F, 0F, 0F};

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                (int) (robot.sensorColor.green() * SCALE_FACTOR),
                (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        return hsvValues[0];
    }
}
