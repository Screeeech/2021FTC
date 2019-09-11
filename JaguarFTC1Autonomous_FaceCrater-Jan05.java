package org.firstinspires.ftc.teamcode;



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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.JsonReaders.JsonReader;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import android.os.Environment;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code assumes that you do NOT have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * The desired path in this example is:
 * - Drive forward for 3 seconds
 * - Spin right for 1.3 seconds
 * - Drive Backwards for 1 Second
 * - Stop and close the claw.
 * <p>
 * The code is written in a simple form with no optimizations.
 * However, there are several ways that this type of sequence could be streamlined,
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "JaguarFTC1Autonomous_FaceCrater", group = "Linear Opmode")
//@Disabled
public class JaguarFTC1Autonomous_FaceCrater extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime();

    // initialize parameters defined in JSON configuration file - JaguarAutoConfig.json
    double ARM_LIFT_POWER = 0.5; // The power for Arm up/down
    double ARM_LIFT_SLOW_POWER = 0.5; // Slow down arm lift when closing to the topmost
    double MIN_POT_VAL = 0.375; // Test result when the arm is at the lowest point
    double MAX_POT_VAL = 0.970; // Test result when the arm is at the topmost point
    double LATCH_OPEN_POS = 1.0;     // The servo position to open latch
    double LATCH_CLOSE_POS = 0.0;     // The servo position to close latch
    double SPEED = 0.2; // The power/speed in drive by encoder mode [-1.0, 1.0]
    long BREAK_TIME = 1000; // The time to pause between autonomous steps, in milliseconds
    double ARM_STRETCH_POWER = 0.6; // The power for Arm forward/backward
    private static final double SYNC_KP = 0.001;               //this value needs to be tuned

    boolean latchOpen = false;
    //static final double START_SPEED = 0.01; // The Slew Rate Code starting power
    //static final double SPEED_INCREMENT = 0.005; // The incremental value at each speed increment step

    private DcMotor baseLeftMotor;
    private DcMotor baseRightMotor;
    private DcMotor armLiftLeftDrive = null; // Left Arm Lift motor
    private DcMotor armLiftRightDrive = null; // Right Arm Lift motor
    private DcMotor armLeftMotor = null;
    private DcMotor armRightMotor = null;
    private Servo latchServo; // Latch servo
    private CRServo intakeServo; // Intake servo
    private AnalogInput armLiftPotentiometer; //Potentiometer to measure Arm lift angle

    enum DrivingDirection {FORWARD, BACKWARD, LEFTTURN, RIGHTTURN;}

    public void initRobot() throws InterruptedException{

        // read data from a JSON configuration file - JaguarAutoConfig.json
        String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Download";
        JsonReader opmodeCfgs = new JsonReader(path+"/JaguarAutoConfig.json");
            /*
            if( opmodeCfgs == null ) {
                telemetry.addData("Debug" , "opmodeCfgs is null : path" + path);
                telemetry.update();
            }
            */

        try {
                //telemetry.addData("debug", "before getJsonArray for JaguarMoves");
                //telemetry.update();

            JSONArray cfgArray = opmodeCfgs.jsonRoot.getJSONArray("JaguarMoves");
            for(int i=0; i<cfgArray.length(); i++) {
                JSONObject move = cfgArray.getJSONObject(i);
                ARM_LIFT_POWER = Double.parseDouble(move.getString("ARM_LIFT_POWER"));
                ARM_LIFT_SLOW_POWER = Double.parseDouble(move.getString("ARM_LIFT_SLOW_POWER"));
                MIN_POT_VAL = Double.parseDouble(move.getString("MIN_POT_VAL"));
                MAX_POT_VAL = Double.parseDouble(move.getString("MAX_POT_VAL"));
                LATCH_OPEN_POS = Double.parseDouble(move.getString("LATCH_OPEN_POS"));
                LATCH_CLOSE_POS = Double.parseDouble(move.getString("LATCH_CLOSE_POS"));
                SPEED = Double.parseDouble(move.getString("SPEED"));
                BREAK_TIME = Long.parseLong(move.getString("BREAK_TIME "));

            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
        telemetry.addData("ARM_LIFT_SLOW_POWER: ", "%f", ARM_LIFT_SLOW_POWER);
        telemetry.addData("MAX_POT_VAL: ", "%f", MAX_POT_VAL);
        telemetry.addData("LATCH_OPEN_POS: ", "%f", LATCH_OPEN_POS);
        //telemetry.addData("DUMPING_ENCODER_VAL: ", "%d", DUMPING_ENCODER_VAL);

        // Get motors ready
        baseLeftMotor = hardwareMap.get(DcMotor.class, "baseLeftMotor");
        baseRightMotor = hardwareMap.get(DcMotor.class, "baseRightMotor");
        armLiftLeftDrive = hardwareMap.get(DcMotor.class, "armLiftLeftMotor");
        armLiftRightDrive = hardwareMap.get(DcMotor.class, "armLiftRightMotor");
        armLeftMotor = hardwareMap.get(DcMotor.class,"armLeftMotor");
        armRightMotor = hardwareMap.get(DcMotor.class, "armRightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        baseLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        baseRightMotor.setDirection(DcMotor.Direction.REVERSE);
        armLiftLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        armLiftRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        armRightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Arm Lift Encoder: ", "Left %d, Right %d", armLiftLeftDrive.getCurrentPosition(), armLiftRightDrive.getCurrentPosition());
        telemetry.addData("Arm Encoder: ", "Left %d, Right %d", armLeftMotor.getCurrentPosition(), armRightMotor.getCurrentPosition());

        baseLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        baseRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // set the current encoder position to zero
        baseLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run at targeted velocity (constant speed)
        // ****** Start and stop will be too rough at this mode ******
        //baseLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //baseRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get servo ready
        intakeServo = hardwareMap.get(CRServo.class, "intakeSvo");
        latchServo = hardwareMap.get(Servo.class, "latchServo");
        //latchServo.setPosition(LATCH_CLOSE_POS);
        latchOpen = false;

        // Get potentiometer ready
        armLiftPotentiometer = hardwareMap.get(AnalogInput.class, "armLiftPotentiometer");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void openLatch()
    {
        latchServo.setPosition(LATCH_OPEN_POS);
        latchOpen = true;
        telemetry.addData("Latch Opened", "");
        telemetry.update();
    }

    public void landing()
    {
        double potVal = 0;

        potVal = armLiftPotentiometer.getVoltage();

        // TODO: Add a timer? In case the arm is stuck and potVal never reaches the MAX_POT_VAL.
        while (opModeIsActive() && potVal<=MAX_POT_VAL) { // Going Up if the arm has not reached the topmost yet
            telemetry.addData("Landing - Potentiometer: ", "%5.3f", potVal);
            telemetry.update();
            // TODO: Do we really need to use SLOW POWER? After the base wheels touch the floor,
            //       we may need to add power to push the robot forward.
            if (potVal>=MAX_POT_VAL*1.2) { // Slow down if the base is close to the floor
                armLiftRightDrive.setPower(ARM_LIFT_SLOW_POWER);
                armLiftLeftDrive.setPower(ARM_LIFT_SLOW_POWER);
            }
            if (potVal<MAX_POT_VAL*0.8) {
                armLiftRightDrive.setPower(ARM_LIFT_SLOW_POWER);
                armLiftLeftDrive.setPower(ARM_LIFT_SLOW_POWER);
            }

            potVal = armLiftPotentiometer.getVoltage();

        }

        // Keeps some power to hold the arm up after it lands
        armLiftRightDrive.setPower(0.1);
        armLiftLeftDrive.setPower(0.1);

    }

    public void encoderDrive(int drivingDirection,
                             double targetSpeed,
                             int targetEncoderValue)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Make sure the encoders are reset
            // TODO: Test if STOP_AND_RESET_ENCODER will cause rough driving
            //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            int currentLeftEncoderValue = baseLeftMotor.getTargetPosition();
            int currentRightEncoderValue = baseRightMotor.getTargetPosition();

            // Sets the desired encoder target position
            switch (drivingDirection) {
                case 0:
                    baseLeftMotor.setTargetPosition(currentLeftEncoderValue-targetEncoderValue);
                    baseRightMotor.setTargetPosition(currentRightEncoderValue-targetEncoderValue);
                    break;
                case 1:
                    baseLeftMotor.setTargetPosition(currentLeftEncoderValue+targetEncoderValue);
                    baseRightMotor.setTargetPosition(currentRightEncoderValue+targetEncoderValue);
                    break;
                case 2:
                    baseLeftMotor.setTargetPosition(currentLeftEncoderValue+targetEncoderValue);
                    baseRightMotor.setTargetPosition(currentRightEncoderValue-targetEncoderValue);
                    break;
                case 3:
                    baseLeftMotor.setTargetPosition(currentLeftEncoderValue-targetEncoderValue);
                    baseRightMotor.setTargetPosition(currentRightEncoderValue+targetEncoderValue);
                    break;

                default:
                    baseLeftMotor.setTargetPosition(currentLeftEncoderValue-targetEncoderValue);
                    baseRightMotor.setTargetPosition(currentRightEncoderValue-targetEncoderValue);
                    break;
            }


            // Set the motor to attempt to rotate in the correct direction
            baseLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // In RUN_USING_ENCODER mode, the drive is too rough. Need Slew Rate Code.
            // Start to run at Slew Rate. Setting a power level of zero will brake the motor
            //double currentSpeed = START_SPEED;
            //leftMotor.setPower(currentSpeed);
            //rightMotor.setPower(currentSpeed);
            baseLeftMotor.setPower(targetSpeed);
            baseRightMotor.setPower(targetSpeed);

            while (opModeIsActive() &&
                    (baseLeftMotor.isBusy() && baseRightMotor.isBusy())) {
                // Slew Rate Code
                //if (currentSpeed < targetSpeed) {
                //    currentSpeed+=SPEED_INCREMENT;
                //    leftMotor.setPower(currentSpeed);
                //    rightMotor.setPower(currentSpeed);
                //}
                telemetry.addData("Position",  "L: %7d R: %7d",
                        baseLeftMotor.getCurrentPosition(),
                        baseRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            //leftMotor.setPower(0);
            //rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    // PID control to sync two motors.
    //
    // Assumption: The two motors have the same encoder reading. Need to set the encoder to 0 during init.
    //
    public void setSyncPower(DcMotor motor1, DcMotor motor2, double power)
    {
        if (power != 0.0)
        {
            double differentialPower = Range.clip((motor2.getCurrentPosition() - motor1.getCurrentPosition())*SYNC_KP, -1.0, 1.0);
            double motor1Power = power + differentialPower;
            double motor2Power = power - differentialPower;
            double minPower = Math.min(motor1Power, motor2Power);
            double maxPower = Math.max(motor1Power, motor2Power);
            // If one of the powers is either > 1 or < -1, we need to scale down both powers to bring the one that is > 1 or < -1 back to 1 or -1.
            // The scale is always positive. The range is (0, 1].
            double scale = maxPower > 1.0? 1.0/maxPower: minPower < -1.0? -1.0/minPower: 1.0;
            motor1Power *= scale;
            motor2Power *= scale;
            //telemetry.addData("Power","motor1 %.2f, motor2 %.2f", motor1Power, motor2Power);
            motor1.setPower(Range.clip(motor1Power,-1,1));
            motor2.setPower(Range.clip(motor2Power,-1,1));
        } else {
            motor1.setPower(0.0);
            motor2.setPower(0.0);
        }
    }


    private void dropTeamMark()
    {
        // Drive forward to close to depot after landing
        //encoderDrive(0, 0.2, 1400);

        // Lower the arm
        while (armLiftPotentiometer.getVoltage() > 0.8) {
            setSyncPower(armLiftLeftDrive, armLiftRightDrive, -ARM_LIFT_SLOW_POWER);
            sleep(200);
        }
        armLiftRightDrive.setPower(0);
        armLiftLeftDrive.setPower(0);

        // Extend the arm to the depot area
        /*
        while (armRightMotor.getCurrentPosition() < 1800) {
            armLeftMotor.setPower(ARM_STRETCH_POWER);
            armRightMotor.setPower(ARM_STRETCH_POWER);
            sleep(200);
        }
        armLeftMotor.setPower(0);
        armRightMotor.setPower(0);
*/
        // Turn on Intake to drop Team Mark
        intakeServo.setPower(1);
        sleep(2000);
        intakeServo.setPower(0);

        // Pull back the arm
        /*
        while (armRightMotor.getCurrentPosition() > 100) {
            armLeftMotor.setPower(-ARM_STRETCH_POWER);
            armRightMotor.setPower(-ARM_STRETCH_POWER);
            sleep(200);
        }
        armLeftMotor.setPower(0);
        armRightMotor.setPower(0);
*/


    }
	
    public void driveToDepot() throws InterruptedException{

        /*
           After landing, Robot will drive to home (autonomous mode) with combination of directions
           of forward, reverse, left turn and right turn. Data of moving direction,
           motor power and moving distance ( Encoder ) will be read from file : driveToHome.json
           - DZ 11/19/18
         */

        int Direction;
        double Power;
        int Encoder;

        String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/Download";
        JsonReader opmodeCfgs = new JsonReader(path+"/driveToDepot.json");
        //telemetry.addData("Debug", "path = " + path);
        /*
            Direction : where the robot will drive to

                FORWARD     : 0
                REVERSE     : 1
                LEFTTURN    : 2
                RIGHTTURN   : 3

            Power   : how fast the robot will drive
            Encoder : how far the robot will drive
         */
        try {
            /* if (opmodeCfgs == null) {
                telemetry.addData("debug", "opmodeCfgs is null");
                telemetry.update();
            }
            */
            JSONArray cfgArray = opmodeCfgs.jsonRoot.getJSONArray("robotMoves");

            // Check to make sure latch is open

            for(int i=0; i<cfgArray.length(); i++) {
                JSONObject move = cfgArray.getJSONObject(i);
                Direction = Integer.parseInt(move.getString("Direction"));
                Power = Double.parseDouble(move.getString("Power"));
                Encoder = Integer.parseInt(move.getString("Encoder"));

                telemetry.addData("Drive ", "Direction=%2d, Power=%5.2f, Encoder=%5d", Direction, Power, Encoder);
                telemetry.update();
                encoderDrive(Direction,Power,Encoder);
            }
        } catch (JSONException e) {
            e.printStackTrace();}

        //encoderDrive(DrivingDirection.FORWARD, SPEED, 300);
        //encoderDrive(DrivingDirection.RIGHTTURN, SPEED, 525);
        //encoderDrive(DrivingDirection.FORWARD, SPEED, 2800);
        //encoderDrive(DrivingDirection.RIGHTTURN, SPEED, 690);
        //encoderDrive(DrivingDirection.FORWARD, 0.4, 2100);
    }

    private void driveToCrater()
    {
        // Drive backward to the Crater
        encoderDrive(1, 0.4, 4000);
	}
	
    @Override
    public void runOpMode() throws InterruptedException{
        //runtime.reset();

        initRobot();

        waitForStart();

        // Landing
        landing();
        sleep(1000);

        // Open Latch
        openLatch();
        sleep(BREAK_TIME);

        // Drive to the Depot
        driveToDepot();
        //sleep(BREAK_TIME);
		
        // Drop the Team Mark
        dropTeamMark();
        //sleep(BREAK_TIME);

        // Drive to the Crater
        driveToCrater();
    }
}
