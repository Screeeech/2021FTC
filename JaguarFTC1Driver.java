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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

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
public class JaguarFTC1Driver
{
    /* Public OpMode members. */
    /* Declare OpMode members. */
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //        (WHEEL_DIAMETER_INCHES * 3.1415);

    // For testBot, Core Hex Motors
    static final double COUNTS_PER_INCH = 50;
    static final double COUNTS_PER_INCH_SIDEWAY = 50;

    /* local OpMode members. */
    JaguarFTC1Bot robot = null;
    LinearOpMode opmode = null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public JaguarFTC1Driver(JaguarFTC1Bot arobot, LinearOpMode aopmode){
        robot = arobot;
        opmode = aopmode;
    }


    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, globalAngle, gain = .15;

        globalAngle = getGlobalAngle();

        if (globalAngle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -globalAngle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getGlobalAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        // getAngularOrientation returns 0 to +180 to -179 to 0 when turning CCW, 0 to -180 to +179 to 0 when turning CW
        // globalAngle will keep increasing when turning CCW or keep decreasing when turning CW until resetAngle() is called.

        Orientation currentAngles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = currentAngles.firstAngle - robot.lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robot.globalAngle += deltaAngle;

        robot.lastAngles = currentAngles;

        return robot.globalAngle;
    }

    /**
     * Method to drive sideway, based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDriveSidewayRight(double speed,
                                      double distance) {

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
        double correction=0;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            // Reset gyro
            robot.resetAngle();

            // Four wheel Mecanum motor determine new target position, and pass to motor controller

            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
                    robot.baseFrontLeftMotor.getCurrentPosition(),
                    robot.baseBackLeftMotor.getCurrentPosition(),
                    robot.baseFrontRightMotor.getCurrentPosition(),
                    robot.baseBackRightMotor.getCurrentPosition());
//            opmode.telemetry.update();

            moveCounts = (int) (distance * COUNTS_PER_INCH_SIDEWAY);
            newFrontLeftTarget = robot.baseFrontLeftMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.baseFrontRightMotor.getCurrentPosition() - moveCounts;
            newBackLeftTarget = robot.baseBackLeftMotor.getCurrentPosition() - moveCounts;
            newBackRightTarget = robot.baseBackRightMotor.getCurrentPosition() + moveCounts;

            // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
            robot.baseFrontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.baseFrontRightMotor.setTargetPosition(newFrontRightTarget);
            robot.baseBackLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.baseBackRightMotor.setTargetPosition(newBackRightTarget);

            robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            opmode.telemetry.addData("Target", "L: %7d %7d R: %7d %7d",
                    newFrontLeftTarget,
                    newBackLeftTarget,
                    newFrontRightTarget,
                    newBackRightTarget);
            opmode.telemetry.update();
            opmode.sleep(1000);

            // Dylan - Four wheel motor start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.baseBackRightMotor.setPower(speed);
            robot.baseFrontLeftMotor.setPower(speed);
            robot.baseBackLeftMotor.setPower(speed);
            robot.baseFrontRightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while ( robot.baseBackRightMotor.isBusy() && robot.baseFrontLeftMotor.isBusy() &&
                    robot.baseBackLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy()) {

                // Use gyro to drive in a straight line.
                correction = checkDirection();

//                opmode.telemetry.addData("1 gyro heading ", robot.lastAngles.firstAngle);
//                opmode.telemetry.addData("2 global heading ", robot.globalAngle);
//                opmode.telemetry.addData("3 correction ", correction);
//                opmode.telemetry.update();

//                robot.baseFrontLeftMotor.setPower(speed - correction);
//                robot.baseFrontRightMotor.setPower(speed + correction);
                robot.baseBackLeftMotor.setPower(speed + correction);
                robot.baseBackRightMotor.setPower(speed + 2*correction);

            }
//            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
//                    robot.baseFrontLeftMotor.getCurrentPosition(),
//                    robot.baseBackLeftMotor.getCurrentPosition(),
//                    robot.baseFrontRightMotor.getCurrentPosition(),
//                    robot.baseBackRightMotor.getCurrentPosition());
//            opmode.telemetry.update();

            // Dylan - four wheel Stop all motion;
            robot.baseFrontLeftMotor.setPower(0);
            robot.baseFrontRightMotor.setPower(0);
            robot.baseBackLeftMotor.setPower(0);
            robot.baseBackRightMotor.setPower(0);
                opmode.telemetry.addData("BR Motor Power ", speed+2*correction);
                opmode.telemetry.update();

            // Dylan - four wheel Turn off RUN_TO_POSITION
            robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
        double correction;

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {
            robot.resetAngle();

            // Four wheel Mecanum motor determine new target position, and pass to motor controller
            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
                    robot.baseFrontLeftMotor.getCurrentPosition(),
                    robot.baseBackLeftMotor.getCurrentPosition(),
                    robot.baseFrontRightMotor.getCurrentPosition(),
                    robot.baseBackRightMotor.getCurrentPosition());
            //opmode.telemetry.update();

            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newFrontLeftTarget = robot.baseFrontLeftMotor.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.baseFrontRightMotor.getCurrentPosition() + moveCounts;
            newBackLeftTarget = robot.baseBackLeftMotor.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.baseBackRightMotor.getCurrentPosition() + moveCounts;

            // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
            robot.baseFrontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.baseFrontRightMotor.setTargetPosition(newFrontRightTarget);
            robot.baseBackLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.baseBackRightMotor.setTargetPosition(newBackRightTarget);

            robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            opmode.telemetry.addData("Target", "L: %7d %7d R: %7d %7d",
                    newFrontLeftTarget,
                    newBackLeftTarget,
                    newFrontRightTarget,
                    newBackRightTarget);
            opmode.telemetry.update();
            opmode.sleep(2000);

            // Dylan - Four wheel motor start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.baseFrontLeftMotor.setPower(speed);
            robot.baseFrontRightMotor.setPower(speed);
            robot.baseBackLeftMotor.setPower(speed);
            robot.baseBackRightMotor.setPower(speed);

            /*
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
             */

            while (opmode.opModeIsActive() &&
                    (robot.baseFrontLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy() &&
                            robot.baseBackLeftMotor.isBusy() && robot.baseBackRightMotor.isBusy())) {

                // Use gyro to drive in a straight line.
                correction = checkDirection();

                opmode.telemetry.addData("1 gyro heading ", robot.lastAngles.firstAngle);
                opmode.telemetry.addData("2 global heading ", robot.globalAngle);
                opmode.telemetry.addData("3 correction ", correction);
                opmode.telemetry.update();

                robot.baseFrontLeftMotor.setPower(speed - correction);
                robot.baseFrontRightMotor.setPower(speed + correction);
                robot.baseBackLeftMotor.setPower(speed - correction);
                robot.baseBackRightMotor.setPower(speed + correction);

            }

            // Dylan - four wheel Stop all motion;
            robot.baseFrontLeftMotor.setPower(0);
            robot.baseFrontRightMotor.setPower(0);
            robot.baseBackLeftMotor.setPower(0);
            robot.baseBackRightMotor.setPower(0);

            // Dylan - four wheel Turn off RUN_TO_POSITION
            robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    /*
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
     */

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

    /*
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        //robot.leftDrive.setPower(0);
        //robot.rightDrive.setPower(0);


        // Dylan - four wheel Stop all motion;
        baseFrontLeftMotor.setPower(0);
        baseFrontRightMotor.setPower(0);
        baseBackLeftMotor.setPower(0);
        baseBackLeftMotor.setPower(0);
    }
    */

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
    /*
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


        // Send desired speeds to motors.
        //robot.leftDrive.setPower(leftSpeed);
        // robot.rightDrive.setPower(rightSpeed);


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
    */

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    /*
    public double getError(double targetAngle) {


        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        double robotError;

        telemetry.addData("deltaAngle", "%5.2f", deltaAngle);
        telemetry.update();
        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - gyro.getIntegratedZValue();
        robotError = targetAngle - deltaAngle;
        telemetry.addData("robotError", "%5.2f", robotError);
        telemetry.update();
        while (robotError <= -180) robotError += 360;
        return robotError;
        //return 0;
    }

    */

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Encoder counts
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    /*
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    */
}

