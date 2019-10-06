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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class JaguarFTCPushbotAutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //        (WHEEL_DIAMETER_INCHES * 3.1415);

    // For testBot, Core Hex Motors
    static final double COUNTS_PER_INCH = 50;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    public BNO055IMU gyro = null;
    //public ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    // Dylan - set for four wheel motor used by team year 2019
    public DcMotor baseFrontLeftMotor = null; // Front Left base motor port 0
    public DcMotor baseFrontRightMotor = null; // Front Right base motor port 1
    public DcMotor baseBackLeftMotor = null; // Back Left base motor port 2
    public DcMotor baseBackRightMotor = null; // Back Right base motor port 3

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robotInit();
/*
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }
*/
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // comment out code from the example
        // robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Dylan -
        baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        waitForStart();

        /*
        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.addData(">", "Robot Heading = %d",gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();
        */

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        telemetry.addData("Prepare to drive FWD", "");
        telemetry.update();
        sleep(3000);
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        // gyroTurn(TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        //gyroHold(TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        //gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        //gyroTurn(TURN_SPEED, 45.0);         // Turn  CW  to  45 Degrees
        //gyroHold(TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        //gyroTurn(TURN_SPEED, 0.0);         // Turn  CW  to   0 Degrees
        //gyroHold(TURN_SPEED, 0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        telemetry.addData("Prepare to drive REV", "");
        telemetry.update();
        sleep(3000);
        gyroDrive(DRIVE_SPEED, -48.0, 0.0);    // Drive REV 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        //int newLeftTarget;
        //int newRightTarget;

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

           /*
            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            */
/*
            telemetry.addData("Position",  "L: %7d %7d R: %7d %7d",
                    baseFrontLeftMotor.getCurrentPosition(),
                    baseBackLeftMotor.getCurrentPosition(),
                    baseFrontRightMotor.getCurrentPosition(),
                    baseBackRightMotor.getCurrentPosition());
            telemetry.update();
*/
            // Dylan - Four wheel motor Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newFrontLeftTarget = baseFrontLeftMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = baseFrontRightMotor.getCurrentPosition() + moveCounts;
            newBackLeftTarget = baseBackLeftMotor.getCurrentPosition() + moveCounts;
            newBackRightTarget = baseBackRightMotor.getCurrentPosition() + moveCounts;

            // Dylan - Four wheel motor Set Target and Turn On RUN_TO_POSITION
            baseFrontLeftMotor.setTargetPosition(newFrontLeftTarget);
            baseFrontRightMotor.setTargetPosition(newFrontRightTarget);
            baseBackLeftMotor.setTargetPosition(newBackLeftTarget);
            baseBackRightMotor.setTargetPosition(newBackRightTarget);

            baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
/*
            telemetry.addData("Target",  "L: %7d %7d R: %7d %7d",
                    newFrontLeftTarget,
                    newBackLeftTarget,
                    newFrontRightTarget,
                    newBackRightTarget);
            telemetry.update();
            sleep(10000);
*/
            // Dylan - Four wheel motor start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            baseFrontLeftMotor.setPower(speed);
            baseFrontRightMotor.setPower(speed);
            baseBackLeftMotor.setPower(speed);
            baseBackRightMotor.setPower(speed);


            /*
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
             */

            while (opModeIsActive() &&
                    (baseFrontLeftMotor.isBusy() && baseFrontRightMotor.isBusy() &&
                            baseBackLeftMotor.isBusy() && baseBackRightMotor.isBusy() )) {
                // adjust relative speed based on heading error.
/*                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                //robot.LeftDrive.setPower(leftSpeed);
                //robot.RightDrive.setPower(rightSpeed);

                baseFrontLeftMotor.setPower(leftSpeed);
                baseFrontRightMotor.setPower(rightSpeed);
                baseBackLeftMotor.setPower(leftSpeed);
                baseBackRightMotor.setPower(rightSpeed);
*/
                /*
                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
                */

                // Dylan - Four wheel Display drive status for the driver.
//                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d:%7d:%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Actual", "%7d:%7d:%7d:%7d", baseFrontLeftMotor.getCurrentPosition(),
                        baseFrontRightMotor.getCurrentPosition(), baseBackLeftMotor.getCurrentPosition(),
                        baseBackRightMotor.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            /*
            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             */

            // Dylan - four wheel Stop all motion;
            baseFrontLeftMotor.setPower(0);
            baseFrontRightMotor.setPower(0);
            baseBackLeftMotor.setPower(0);
            baseBackRightMotor.setPower(0);

            // Dylan - four wheel Turn off RUN_TO_POSITION
            baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        /*
        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
         */

        // Dylan - four wheel Stop all motion;
        baseFrontLeftMotor.setPower(0);
        baseFrontRightMotor.setPower(0);
        baseBackLeftMotor.setPower(0);
        baseBackLeftMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        /*
        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);
         */

        // Dylan - four wheel Send desired speeds to motors
        baseFrontLeftMotor.setPower(leftSpeed);
        baseFrontRightMotor.setPower(rightSpeed);
        baseBackLeftMotor.setPower(leftSpeed);
        baseBackRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - gyro.getIntegratedZValue();
        robotError = targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
        return 0;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Encoder counts
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void robotInit()
    {
/*
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


        // Get motors for base and arm ready
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Dylan - Declare four wheel motor
        baseFrontLeftMotor = hardwareMap.get(DcMotor.class, "baseFrontLeftMotor");
        baseFrontRightMotor = hardwareMap.get(DcMotor.class, "baseFrontRightMotor");
        baseBackLeftMotor = hardwareMap.get(DcMotor.class, "baseBackLeftMotor");
        baseBackRightMotor = hardwareMap.get(DcMotor.class, "baseBackRightMotor");
//        armLiftLeftDrive = hardwareMap.get(DcMotor.class, "armLiftLeftMotor");
//        armLiftRightDrive = hardwareMap.get(DcMotor.class, "armLiftRightMotor");
//        armLeftMotor = hardwareMap.get(DcMotor.class, "armLeftMotor");
//        armRightMotor = hardwareMap.get(DcMotor.class, "armRightMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
/*
        baseFrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        baseFrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        baseBackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        baseBackRightMotor.setDirection(DcMotor.Direction.FORWARD);
*/
        // For testBot, Core Hex Motors
        baseFrontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        baseFrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        baseBackLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        baseBackRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Dylan - Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        baseFrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseFrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseBackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseBackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        armLiftLeftDrive.setDirection(DcMotor.Direction.FORWARD);
//        armLiftRightDrive.setDirection(DcMotor.Direction.REVERSE);
//        armLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        armRightMotor.setDirection(DcMotor.Direction.REVERSE);

//        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
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

        telemetry.addData("Mode", "gyro calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !gyro.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("gyro calib status", gyro.getCalibrationStatus().toString());
        telemetry.update();



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
//        telemetry.addData("Arm Lift Encoder: ", "Left %d, Right %d", armLiftLeftDrive.getCurrentPosition(), armLiftRightDrive.getCurrentPosition());

//        armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
//        telemetry.addData("Arm Encoder: ", "Left %d, Right %d", armLeftMotor.getCurrentPosition(), armRightMotor.getCurrentPosition());

        // set the current encoder position to zero
        // NOTE: Not need to set encoder to 0. It is set to 0 by default.
        //armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get servo ready
//        intakeServo = hardwareMap.get(CRServo.class, "intakeSvo");
//        latchServo = hardwareMap.get(Servo.class, "latchServo");
//        secondlatch = hardwareMap.get(Servo.class, "secondlatch");
        //latchServo.setPosition(LATCH_OPEN_POS);
        //latchOpen = true;

        // Get SPARKMini motor controller (Test SPARKMini)
        //armLeftLiftMotor  = hardwareMap.get(DcMotorSimple.class, "armLeftLiftMotor");
        //armRightLiftMotor = hardwareMap.get(DcMotorSimple.class, "armRightLiftMotor");
        //armLeftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //armRightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Get potentiometer ready
//        armLiftPotentiometer = hardwareMap.get(AnalogInput.class, "armLiftPotentiometer");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}