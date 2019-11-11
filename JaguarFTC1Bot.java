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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
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
    // Public OpMode members.
    public BNO055IMU gyro = null;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;

    // set for four wheel Mecanum motor used by team
    public DcMotor baseFrontLeftMotor = null; // Front Left base motor
    public DcMotor baseFrontRightMotor = null; // Front Right base motor
    public DcMotor baseBackLeftMotor = null; // Back Left base motor
    public DcMotor baseBackRightMotor = null; // Back Right base motor

    // lift motors
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;

    // sensors
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;
    public TouchSensor sensorTouch = null;

    // servos
    public Servo slideServo = null;
    public Servo clawServo = null;
    public Servo clawheadServo = null;

    // robot state constants
    private boolean testBot = false;  //motors will not be initialized if true

    static final double LATCH_OPEN_POS = 0.7;     // The servo position to open latch
    static final double LATCH_CLOSE_POS = 0.0;  // The servo position to close latch
    boolean clawOpen = true;
    boolean clawHeadHorizontal = true;

    // local OpMode members
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    //private ElapsedTime period  = new ElapsedTime();

    // Constructor
    public JaguarFTC1Bot(){

    }

    // Initialize standard Hardware interfaces
    public void robotInit(HardwareMap ahwMap, Telemetry atelemetry)
    {
        hardwareMap = ahwMap;
        telemetry = atelemetry;

/*
        // This code has been deprecated since we now rely on TensorFlow for our Autonomous
        // read data from a JSON configuration file - JaguarUserControlConfig.json
        String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Download";

        org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader opmodeCfgs = new org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader(path+"/JaguarUserControlConfig.json");

        try {
            JSONArray cfgArray = opmodeCfgs.jsonRoot.getJSONArray("JaguarFTC1_UserControl_Config");
            for(int i=0; i<cfgArray.length(); i++) {
                JSONObject userControlConfig = cfgArray.getJSONObject(i);
                NORMAL_DRIVE_POWER_FACTOR = Double.parseDouble(userControlConfig.getString("NORMAL_DRIVE_POWER_FACTOR"));
                SLOW_DRIVE_POWER_FACTOR = Double.parseDouble(userControlConfig.getString("SLOW_DRIVE_POWER_FACTOR"));
                ARM_LIFT_POWER = Double.parseDouble(userControlConfig.getString("ARM_LIFT_POWER"));
                ARM_LIFT_SLOW_POWER = Double.parseDouble(userControlConfig.getString("ARM_LIFT_SLOW_POWER"));
                ARM_STRETCH_POWER = Double.parseDouble(userControlConfig.getString("ARM_STRETCH_POWER"));
                ARM_STRETCH_SLOW_POWER = Double.parseDouble(userControlConfig.getString("ARM_STRETCH_SLOW_POWER"));
                MIN_POT_VAL = Double.parseDouble(userControlConfig.getString("MIN_POT_VAL"));
                MAX_POT_VAL = Double.parseDouble(userControlConfig.getString("MAX_POT_VAL"));
                MAX_ARM_ENCODER_VAL = Integer.parseInt(userControlConfig.getString("MAX_ARM_ENCODER_VAL"));
                MIN_ARM_ENCODER_VAL = Integer.parseInt(userControlConfig.getString("MIN_ARM_ENCODER_VAL"));
                DUMPING_POT_VAL = Double.parseDouble(userControlConfig.getString("DUMPING_POT_VAL"));
                DUMPING_ENCODER_VAL = Integer.parseInt(userControlConfig.getString("DUMPING_ENCODER_VAL"));
                HANGING_POT_VAL = Double.parseDouble(userControlConfig.getString("HANGING_POT_VAL"));
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
        // Check if reading config file correctly
        //telemetry.addData("ARM_LIFT_POWER: ", "%f", ARM_LIFT_POWER);
        telemetry.addData("DUMPING_ENCODER_VAL: ", "%d", DUMPING_ENCODER_VAL);
*/

        // Get motors for base and lift ready
        if (!testBot) {
            leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
            rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
            leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // We want lift motor to run at the giving power, not the giving speed. So we can control the holding power
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        baseFrontLeftMotor = hardwareMap.get(DcMotor.class, "baseFrontLeftMotor");
        baseFrontRightMotor = hardwareMap.get(DcMotor.class, "baseFrontRightMotor");
        baseBackLeftMotor = hardwareMap.get(DcMotor.class, "baseBackLeftMotor");
        baseBackRightMotor = hardwareMap.get(DcMotor.class, "baseBackRightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        if (!testBot) {
            baseFrontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            baseFrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            baseBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            baseBackRightMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else {
            // For testBot, Core Hex Motors
            baseFrontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            baseFrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            baseBackLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            baseBackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        baseFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run at constant speed
        baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that is built-into the color sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // get a reference to the touch sensor
        // if you are having problems with this, know that touch sensors must be in the n+1 field
        sensorTouch = hardwareMap.get(TouchSensor.class, "touchsensor");

        // Get servo ready
        slideServo = hardwareMap.get(Servo.class, "slideServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawheadServo = hardwareMap.get(Servo.class, "clawheadServo");

        // Update booleans
        clawOpen = true;
        clawHeadHorizontal = true;

        // Close latch
        slideServo.setPosition(LATCH_CLOSE_POS);
        //clawServo.setDirection(Servo.Direction.REVERSE);
        //clawServo.setPosition(LATCH_OPEN_POS);
        //clawheadServo.setDirection(Servo.Direction.FORWARD);
        //clawheadServo.setPosition(LATCH_OPEN_POS);

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

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!gyro.isGyroCalibrated())
        {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            Thread.yield();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
    }

    //Resets the cumulative angle tracking to zero.
    public void resetAngle()
    {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
 }

