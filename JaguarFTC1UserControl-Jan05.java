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

import android.os.Environment;
import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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

@TeleOp(name="JaguarFTC1UserControl", group="Linear Opmode")
//@Disabled
public class JaguarFTC1UserControl extends LinearOpMode {

    double NORMAL_DRIVE_POWER_FACTOR = 0.7; // The power factor for normal drive
    double SLOW_DRIVE_POWER_FACTOR = 0.2; // The power factor for slow drive
    double ARM_LIFT_POWER = 0.7; // The power for Arm up/down
	double ARM_LIFT_SLOW_POWER = 0.4; // Slow down arm lift when closing to the topmost
	double ARM_STRETCH_POWER = 0.8; // The power for Arm forward/backward
	double ARM_STRETCH_SLOW_POWER = 0.2; // Slow down arm stretch when closing to the longest
    double MIN_POT_VAL = 0.375; // Test result when the arm is at the lowest point
    double MAX_POT_VAL = 0.975; // Test result when the arm is at the topmost point
	int MAX_ARM_ENCODER_VAL = 2100; // Test result when the arm is stretched to the longest. Left: 1750, Right: 2100
	int MIN_ARM_ENCODER_VAL = 0; //The encoder value when the arm is fully pulled back. Always 0 after init.
    double DUMPING_POT_VAL = 0.6; // Potentiometer value for the intake to reach the Lander edge before dumping minerals
    int DUMPING_ENCODER_VAL = 1000; // The arm encoder value for the intake to reach teh Lander edge before dumping minerals
    double HANGING_POT_VAL = 0.5; // Potentiometer value for robot to hang
    double ARM_LIFT_HOLDING_POWER = 0.1;
    double HOLDING_POT_VAL = 0.85;

    static final double TOLERANCE = 0.05;
	static final double LATCH_OPEN_POS = 1.0;     // The servo position to open latch
	static final double LATCH_CLOSE_POS = 0.0;  // The servo position to close latch
    static final double SECOND_LATCH_OPEN_POS = 0.0;
    static final double SECOND_LATCH_CLOSE_POS = 1.0;
    private static final double SYNC_KP = 0.001;               //this value needs to be tuned
    //private static final double ENCODER_DIFF_TOLERANCE = 50; //this value needs to be tuned
	boolean latchOpen = true;
	boolean secondLatchOpen = true;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor baseLeftDrive = null; // Left base motor
    private DcMotor baseRightDrive = null; // Right base motor
    private DcMotor armLiftLeftDrive = null; // Left Arm Lift motor
    private DcMotor armLiftRightDrive = null; // Right Arm Lift motor
    private DcMotor armLeftMotor = null; // Left Arm motor
    private DcMotor armRightMotor = null; // Right Arm motor
	private Servo   latchServo = null; // Latch servo
    private CRServo   intakeServo = null; // Intake servo
	private AnalogInput armLiftPotentiometer = null; //Potentiometer to measure Arm lift angle
    private Servo secondlatch = null;

    enum IntakeRollerStatus {ROLL_IN, STOPPED, ROLL_OUT}

    @Override
    public void runOpMode() {
		double potVal = 0; // Keep potentiometer reading (Arm angle)
		int armLeftMotor_encoderVal = 0; // Keep left Arm Stretch motor encoder reading (Arm length)
		int armRightMotor_encoderVal = 0; // Keep right Arm Stretch motor encoder reading (Arm length)
		boolean latchActionFinished = true; // Keep if Latch opening/closing has finished (Latch button has been released)
        boolean secondLatchActionFinished = true;
        boolean intakeButtonReleased = true; // Keep if Intake action has finished (Latch button has been released)
		IntakeRollerStatus intakeRollerStatus = IntakeRollerStatus.STOPPED; // Intake status
		boolean autoLiftIsGoing = false; // Keep if Auto Lift is on going. It is set to false after Auto Lift finished. Or set to false to break Auto Lift
		boolean autoLiftUpDownReady = true;
		boolean autoLiftOutBackReady = true;
		boolean autoHangIsGoing = false; // Keep if Auto Hang is on going. It is set to false after Auto Hang finished. Or set to false to break Auto Hang

		robotInit();
		
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
			
            // Base Control               
			//                     <<<<< Left Stick Y and Right stick X >>>>>
			//
            // POV Mode uses left stick to go forward, and right stick to turn. Easier to drive straight.
            double x = gamepad1.left_stick_y; // Forward and backward
            //double y =  gamepad1.left_stick_x; // Sideway left and right
            double r =  -gamepad1.right_stick_x; //Turn left and right

            if(Math.abs(x) > TOLERANCE || Math.abs(r) > TOLERANCE ) {
                // y to move to goldilocks height
                // right trigger to move everything slower
                manualDrive(x,r);
            }
            else {
                baseLeftDrive.setPower(0);
                baseRightDrive.setPower(0);
            }


            // Arm Lift Control           
			//                      <<<<< Left Trigger and Right Trigger >>>>>
			//
			// TODO: May need to add holding power after intake is attached.
			//       The intake is too heavy. The arm may not be able to keep up by itself.
            potVal = armLiftPotentiometer.getVoltage();
            telemetry.addData("Potentiometer: ", "%5.3f", potVal);
            telemetry.addData("Arm Lift Encoder: ", "Left %d, Right %d", armLiftLeftDrive.getCurrentPosition(), armLiftRightDrive.getCurrentPosition());
            if (gamepad1.left_trigger>0.8 && potVal<=MAX_POT_VAL) { // Going Up if the arm has not reached the topmost yet
                if (autoLiftIsGoing) autoLiftIsGoing = false; // Break Auto Lift if Auto Lift is going on.
                if (autoHangIsGoing) autoHangIsGoing = false; // Break Auto Hang if Auto Hang is going on.

				if (potVal>=(MAX_POT_VAL*0.8)) { // Slow down if close to the topmost
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, ARM_LIFT_SLOW_POWER);
				}
				else {
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, ARM_LIFT_POWER);
				}
            }
            else if (gamepad1.right_trigger>0.8 && potVal>=MIN_POT_VAL){ // Going Down if the arm has not reached the bottom yet
                if (autoLiftIsGoing) autoLiftIsGoing = false; // Break Auto Lift if Auto Lift is going on.
                if (autoHangIsGoing) autoHangIsGoing = false; // Break Auto Hang if Auto Hang is going on.

                // TODO: Need to use encoder to keep the arm moving down at constant speed?
                //       Otherwise, the arm may hit the stop bar too hard.
                if (potVal<=(MIN_POT_VAL*1.2)) { // Slow down if close to the bottom
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, -ARM_LIFT_SLOW_POWER);
                }
                else {
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, -ARM_LIFT_POWER);
                }
            }
            else {
                // Intake is heavy. Without holding power the Arm will drop a little bit. Hard to move the Latch into the Lander handle.
                if (potVal>=HOLDING_POT_VAL) {
                    armLiftRightDrive.setPower(ARM_LIFT_HOLDING_POWER);
                    armLiftLeftDrive.setPower(ARM_LIFT_HOLDING_POWER);
                }
                else {
                    armLiftRightDrive.setPower(0);
                    armLiftLeftDrive.setPower(0);
                }

            }



            // Arm Stretch Control
            //                    <<<<< DPAD Up and DPAD Down >>>>>>
            //
            // NOTE: The Left Arm Motor encoder value doesn't change at the same rate as the value of Right Arm Motor encoder
            //       even though the two motors are connected by shaft.
            //       Currently, the ticks per round of right motor are larger than those of left motor.
            //       We just use Right Arm Motor encoder value to make decision.
            armLeftMotor_encoderVal = armLeftMotor.getCurrentPosition();
            armRightMotor_encoderVal = armRightMotor.getCurrentPosition();
            telemetry.addData("Arm Stretch Encoder: ", "Left %5d, Right %5d", armLeftMotor_encoderVal, armRightMotor_encoderVal);

            if (gamepad1.dpad_up) {
                if (autoLiftIsGoing) autoLiftIsGoing = false; // Break Auto Lift if Auto Lift is going on.

                if (armRightMotor_encoderVal>=(MAX_ARM_ENCODER_VAL*0.9)) { // Slow down if close to the longest
                    armLeftMotor.setPower(ARM_STRETCH_SLOW_POWER);
                    armRightMotor.setPower(ARM_STRETCH_SLOW_POWER);
                }
                else {
                    armLeftMotor.setPower(ARM_STRETCH_POWER);
                    armRightMotor.setPower(ARM_STRETCH_POWER);
                }
            }
            else if (gamepad1.dpad_down) {
                if (autoLiftIsGoing) autoLiftIsGoing = false; // Break Auto Lift if Auto Lift is going on.

                if (armRightMotor_encoderVal<=(MIN_ARM_ENCODER_VAL+200)) { // Slow down if almost pulled back
                    armLeftMotor.setPower(-ARM_STRETCH_SLOW_POWER);
                    armRightMotor.setPower(-ARM_STRETCH_SLOW_POWER);
                }
                else {
                    armLeftMotor.setPower(-ARM_STRETCH_POWER);
                    armRightMotor.setPower(-ARM_STRETCH_POWER);
                }
            }
            else {
                armLeftMotor.setPower(0);
                armRightMotor.setPower(0);
            }



            // Auto lift Arm to the height that the Intake can touch Lander edge for dumping minerals
            //                     <<<<< Start + Y >>>>>
            //
            // TODO: Need to check if the robot has attached to the Lander (Latch is close?). If we accidentally press Auto Lift buttons
            // when we actually want to Auto Hang, we may damage the Arm since the Arm will extend during Auto Lift.
            if (gamepad1.start && gamepad1.y) {
                autoLiftUpDownReady = false;
                autoLiftOutBackReady = false;
                autoLiftIsGoing = true;
            }

            if (autoLiftIsGoing) {
                // If Arm angle is too large, we need to lower the Arm. If too small, we need to lift the Arm.
                // We need to check if the Arm angle is within a range, for example, (0.95*ExpectedAngle, 1.1*ExpectedAngle).
                // Don't expect the angle will be exactly at the desired angle. Otherwise, the Arm will oscillate.
                if (potVal < DUMPING_POT_VAL*0.95) { // Going UP
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, ARM_LIFT_POWER*0.8);
                }
                else if (potVal > DUMPING_POT_VAL*1.1) { // Going Down
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, -ARM_LIFT_POWER*0.2);
                }
                else {
                    autoLiftUpDownReady = true;
                    //if (autoLiftOutBackReady)
                    //    autoLiftIsGoing = false;
                    armLiftLeftDrive.setPower(0.2);
                    armLiftRightDrive.setPower(0.2);
                }

                // If Arm is too long, we need to pull back the Arm. If too short, we need to move out the Arm.
                // We need to check if the Arm length is within a range, for example, (0.9*ExpectedLength, 1.1*ExpectedLength).
                // Don't expect the length will be exactly at the desired length. Otherwise, the Arm will oscillate.
                if (armRightMotor_encoderVal < (DUMPING_ENCODER_VAL-50)) { // Arm going out
                    armLeftMotor.setPower(ARM_STRETCH_POWER);
                    armRightMotor.setPower(ARM_STRETCH_POWER);
                }
                else if (armRightMotor_encoderVal > (DUMPING_ENCODER_VAL+50)) { // Arm going back
                    armLeftMotor.setPower(-ARM_STRETCH_POWER);
                    armRightMotor.setPower(-ARM_STRETCH_POWER);
                }
                else {
                    autoLiftOutBackReady = true;
                    //if (autoLiftUpDownReady)
                    //    autoLiftIsGoing = false;
                    armLeftMotor.setPower(0);
                    armRightMotor.setPower(0);
                }
            }


			// Auto Hang
            //                     <<<<< Start + X >>>>>
            // TODO: Use other buttons. "Start + X" may cause confusion with Auto Lift buttons "Start + Y" and damage robot if the robot has attached to Lander
			// Hang the robot automatically

            // TODO: May need to check if the Latch is close. It doesn't hurt robot if the Latch is still open. Just waste time.
            if (gamepad1.start && gamepad1.x) {
                autoHangIsGoing = true;
            }

            if (autoHangIsGoing) {
                // Auto Hang can only be executed when the Arm is at the topmost. So no need to check if potVal < HANGING_POT_VAL.
                if (potVal > 1.3*HANGING_POT_VAL) { // Fast. Arm going Down (Base up)
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, -0.9);
                } else if (potVal > HANGING_POT_VAL) { // Slow. Arm going Down (Base up)
                    setSyncPower(armLiftLeftDrive, armLiftRightDrive, -0.5);
                } else {
                    if (secondLatchOpen) {
                        secondlatch.setPosition(SECOND_LATCH_CLOSE_POS);
                        secondLatchOpen = false;
                    }

                    // Run following code if holding power is not needed
                    //autoHangIsGoing = false;
                    //armLiftLeftDrive.setPower(0);
                    //armLiftRightDrive.setPower(0);

                    armLiftLeftDrive.setPower(-0.2);
                    armLiftRightDrive.setPower(-0.2);
                }
            }

			
			
			// Latch Control
			//                      <<<<< Left Bumper >>>>>
			// Toggle the latch open and close
			//
			// TODO: Latch should not be opened while the robot is being hanged.
			//       Need to check the Arm angle before opening latch.
			//
			// If Left Bumper has been pressed and still being pressed, we consider the latch is performing toggle action now.
			// No further action is needed.
            if (gamepad1.left_bumper && latchActionFinished) {
				latchActionFinished = false;
				// TODO: Use Servo getPosition(), instead of latchOpen, to check if the latch is open or close.
			    if (latchOpen) { // Latch is currently open
				    latchServo.setPosition(LATCH_CLOSE_POS);
				    latchOpen = false;
			    }
			    else { // Latch is currently close
				    latchServo.setPosition(LATCH_OPEN_POS);
				    latchOpen = true;
			    }
			}
			// If Left Bumper has been released, we consider the latch has finished the toggle action.
			// Reset latchActionFinished to TRUE.
			if (!latchActionFinished && !gamepad1.left_bumper)
				latchActionFinished = true;


            // Lock Control
            //                     <<<<< Start + Left Bumper >>>>>
            // Toggle the latch that locks robot during hanging open and close
            //
            if (gamepad1.b && secondLatchActionFinished) {
                secondLatchActionFinished = false;
                if (secondLatchOpen) { // Latch is currently open
                    secondlatch.setPosition(SECOND_LATCH_CLOSE_POS);
                    secondLatchOpen = false;
                }
                else { // Latch is currently close
                    secondlatch.setPosition(SECOND_LATCH_OPEN_POS);
                    secondLatchOpen = true;
                }
            }
            // If Left Bumper has been released, we consider the latch has finished the toggle action.
            // Reset latchActionFinished to TRUE.
            if (!secondLatchActionFinished && !gamepad1.b)
                secondLatchActionFinished = true;


            // Intake Control
            //                <<<<< DPAD Left and DPAD Right >>>>>>
            //
            // Following display is used to get the power that stops the CR servo
            //telemetry.addData("Servo Position", "%5.2f", gamepad1.right_stick_y);
            //intakeServo.setPower(0.05); // The power to stop HSR-1425CR servo

            if (gamepad1.dpad_left) {
                if (intakeButtonReleased) {
                    intakeButtonReleased = false;
                    if (intakeRollerStatus != IntakeRollerStatus.ROLL_IN) {
                        intakeRollerStatus = IntakeRollerStatus.ROLL_IN;
                        intakeServo.setPower(1);
                    } else {
                        intakeRollerStatus = IntakeRollerStatus.STOPPED;
                        intakeServo.setPower(0);
                    }
                }
            }
            else if (gamepad1.dpad_right) {
                if (intakeButtonReleased) {
                    intakeButtonReleased = false;
                    if (intakeRollerStatus != IntakeRollerStatus.ROLL_OUT) {
                        intakeRollerStatus = IntakeRollerStatus.ROLL_OUT;
                        intakeServo.setPower(-1);
                    } else {
                        intakeRollerStatus = IntakeRollerStatus.STOPPED;
                        intakeServo.setPower(0);
                    }
                }
            }
            else
                intakeButtonReleased = true;


            //sleep(CYCLE_MS);
            idle(); // For multiple threads program to give other thread a change to run.
            telemetry.update();

        }
    }

	public void robotInit()
	{
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


        // Get motors for base and arm ready
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseLeftDrive = hardwareMap.get(DcMotor.class, "baseLeftMotor");
        baseRightDrive = hardwareMap.get(DcMotor.class, "baseRightMotor");
        armLiftLeftDrive = hardwareMap.get(DcMotor.class, "armLiftLeftMotor");
        armLiftRightDrive = hardwareMap.get(DcMotor.class, "armLiftRightMotor");
        armLeftMotor = hardwareMap.get(DcMotor.class, "armLeftMotor");
        armRightMotor = hardwareMap.get(DcMotor.class, "armRightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        baseLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        baseRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armLiftLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        armLiftRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        armRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // !!!!!!!!!!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!
        //
        // To sync Lift motors, we need to set their encoder values to be same during init.
        // If the encoder values of the sync motors are not same at the beginning, the setSyncPower will try to bring them same and cause robot damage.
        // Fortunately, FTC resets encoder to 0 by default during motor init.
        // TODO: Arm Lift motor encoder can only be reset after a full power cycle (unplug cable from phone and power cycle Hub).
        //       It won't be reset by just restarting the RC. But Arm Stretch motor encoder can be reset by both restarting.
        //       WHY??????? HD motor and Hex motor encoders are reset differently?
        //
        // !!!!!!!!!!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!

        // Since the encoder is set to zero by default after starting, we don't need to call STOP_AND_RESET_ENCODER explicitly.
        // But need to check the both encoder values by displaying their values.
        // TODO: If encoder is reset, it seems the RC has trouble to connect Hub after robot stopped. Firmware bug?
        //armLiftLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armLiftRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Arm Lift Encoder: ", "Left %d, Right %d", armLiftLeftDrive.getCurrentPosition(), armLiftRightDrive.getCurrentPosition());

        armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Following code is used to manually get the arm motor encoder value when the arm is at the longest.
		// Since the pulley string may loose, the arm motor encoder reading will change.
		// We need to get that value every time before the game.
        // NOTE: The Left Arm Motor encoder value doesn't change at the same rate as the value of Right Arm Motor encoder
        //       even though the two motors are connected by shaft.
        //       Currently, the ticks per round of right motor are larger than those of left motor.
        //       We just use Right Arm Motor encoder value to make decision.
		//
		// ************************************************************************************************
		// HOW TO DO:                                                                                     *
        // 1. Move the Arm all the way back to the end.                                                   *
        // 2. Turn on robot. Press Init on RC app.                                                        *
        // 3. Manually turn arm motor shaft to stretch the arm to its longest.                            *
        // 4. Write down the RIGHT arm motor encoder value and update MAX_ARM_ENCODER_VAL in config file. *
        // 5. Restart robot.                                                                              *
		// ************************************************************************************************
		//
        // TODO: Will the Arm Lift and Arm encoders can still be reset after Autonomous?
		telemetry.addData("Arm Encoder: ", "Left %d, Right %d", armLeftMotor.getCurrentPosition(), armRightMotor.getCurrentPosition());
        
		// set the current encoder position to zero
        // NOTE: Not need to set encoder to 0. It is set to 0 by default.
        //armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get servo ready
        intakeServo = hardwareMap.get(CRServo.class, "intakeSvo");
		latchServo = hardwareMap.get(Servo.class, "latchServo");
		secondlatch = hardwareMap.get(Servo.class, "secondlatch");
		//latchServo.setPosition(LATCH_OPEN_POS);
		//latchOpen = true;

        // Get SPARKMini motor controller (Test SPARKMini)
        //armLeftLiftMotor  = hardwareMap.get(DcMotorSimple.class, "armLeftLiftMotor");
        //armRightLiftMotor = hardwareMap.get(DcMotorSimple.class, "armRightLiftMotor");
        //armLeftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //armRightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		// Get potentiometer ready
		armLiftPotentiometer = hardwareMap.get(AnalogInput.class, "armLiftPotentiometer");
		
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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

    public void manualDrive(double x, double r)
    {
        // Setup a variable for each drive wheel to save power level for telemetry
        double baseLeftPower;
        double baseRightPower;

        baseLeftPower = Range.clip(x + r, -1.0, 1.0) ;
        baseRightPower = Range.clip(x - r, -1.0, 1.0) ;

        // Send calculated power to wheels
        if (gamepad1.right_bumper){ // Slow motion
            baseLeftDrive.setPower(baseLeftPower*SLOW_DRIVE_POWER_FACTOR);
            baseRightDrive.setPower(baseRightPower*SLOW_DRIVE_POWER_FACTOR);
        }
        else {
            baseLeftDrive.setPower(baseLeftPower*NORMAL_DRIVE_POWER_FACTOR);
            baseRightDrive.setPower(baseRightPower*NORMAL_DRIVE_POWER_FACTOR);
        }

        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", baseLeftPower, baseRightPower);
        //telemetry.update();

    }
}
