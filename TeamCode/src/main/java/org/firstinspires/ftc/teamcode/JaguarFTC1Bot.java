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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class JaguarFTC1Bot
{
    /* Public OpMode members. */
    public BNO055IMU gyro = null;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;

    // set for four wheel Mecanum motor used by team
    // TODO: play around with DcMotorExEx for better functionality
    public DcMotorEx baseFrontLeftMotor = null; // Front Left base motor
    public DcMotorEx baseFrontRightMotor = null; // Front Right base motor
    public DcMotorEx baseBackLeftMotor = null; // Back Left base motor
    public DcMotorEx baseBackRightMotor = null; // Back Right base motor

    // Test bot?
    private boolean testBot = false;

    // Sensors

    // Encoder and servo position variables


    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public JaguarFTC1Bot(){

    }

    /* Initialize standard Hardware interfaces */
    public void robotInit(HardwareMap ahwMap, Telemetry atelemetry)
    {
        hardwareMap = ahwMap;
        telemetry = atelemetry;

        // Get motors for base and lift ready
        if (!testBot) {
            // set stuff that only applies to actual robot
        }

        // Base stays the same for all robots
        baseFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "baseFrontLeftMotor");
        baseFrontRightMotor = hardwareMap.get(DcMotorEx.class, "baseFrontRightMotor");
        baseBackLeftMotor = hardwareMap.get(DcMotorEx.class, "baseBackLeftMotor");
        baseBackRightMotor = hardwareMap.get(DcMotorEx.class, "baseBackRightMotor");


        // Make sure all motors face the same direction on both robots
        baseFrontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        baseFrontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        baseBackLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        baseBackRightMotor.setDirection(DcMotorEx.Direction.REVERSE);


        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        baseFrontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        baseFrontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        baseBackLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        baseBackRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // Set to run at constant speed
        baseFrontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        baseFrontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        baseBackLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        baseBackRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        resetAngle();

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        //while (!isStopRequested() && !gyro.isGyroCalibrated())
        while (!gyro.isGyroCalibrated())
        {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            Thread.yield();
        }
        //gyro.startAccelerationIntegration(new Position(), new Velocity(), 500);
        telemetry.addData(">", "Robot Ready.");//
        telemetry.update();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
 }

