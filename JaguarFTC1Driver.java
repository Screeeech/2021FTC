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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
public class JaguarFTC1Driver {
    /* Public OpMode members. */
    /* Declare OpMode members. */
    //static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    //static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    //static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //        (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double VEX_MOTOR_RUN = 0.7;     // The servo position to open latch
    static final double VEX_MOTOR_STOP = 0.0;
    static final double SERVO_SET_POSITION = 1.0;     // The servo set position
    static final double SERVO_INIT_POSITION = 0.0;  // The servo init position

    // For testBot, Core Hex Motors
    static final double COUNTS_PER_INCH_FORWARD = 69.4; //69;
    static final double COUNTS_PER_INCH_BACKWARD = 70.3; //69;
    static final double COUNTS_PER_INCH_SIDEWAY_LEFT = 75.8; //75;
    static final double COUNTS_PER_INCH_SIDEWAY_RIGHT = 76.8; //75;

    static final double LIFT_HOLDING_POWER = 0.2;

    // when the robot moves close to the stones (prior to pickup), the robot stops at this distance from the stones.
    static public final float STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES = 30F;
    static public final int DISTANCE_CODE_CLOSE = 0;
    static public final int DISTANCE_CODE_SLOWDOWN = 1;
    static public final int DISTANCE_CODE_FAR = 2;
    static public final double forwardSpeed = 0.2;
    static public final double slowdownSpeed = 0.1;

    static public final double DGREE_TOLERANCE = 5.0;
    static public final double fastTurnPower = 0.4;
    static public final double slowTurnPower = 0.1;

    /* local OpMode members. */
    JaguarFTC1Bot robot = null;
    LinearOpMode opmode = null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public JaguarFTC1Driver(JaguarFTC1Bot arobot, LinearOpMode aopmode) {
        robot = arobot;
        opmode = aopmode;
    }


    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @param angleRobotToFace, this is the global angle that we want to maintain. During init, the robot is facing 0 degree.
     *                     CCW from 0, the angle is from 0 to 180. CW from 0, the angle is from 0 to -180
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection(float angleRobotToFace) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, globalAngle, gain = 0.01;

        globalAngle = getGlobalAngle();

        if (globalAngle == angleRobotToFace)
            correction = 0;             // no adjustment.
        else
            correction = -(globalAngle-angleRobotToFace);        // reverse sign of angle for correction.

        correction = correction * gain;
        //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
        //opmode.telemetry.update();

        return correction;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getGlobalAngle() {
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
     *                 param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void driveSidewayRight(double speed, double distance, float angleRobotToFace) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        int moveCounts;
        double correction = 0;

        // Four wheel Mecanum motor determine new target position, and pass to motor controller

//            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
//                    robot.baseFrontLeftMotor.getCurrentPosition(),
//                    robot.baseBackLeftMotor.getCurrentPosition(),
//                    robot.baseFrontRightMotor.getCurrentPosition(),
//                    robot.baseBackRightMotor.getCurrentPosition());
//            opmode.telemetry.update();

        moveCounts = (int) (distance * COUNTS_PER_INCH_SIDEWAY_RIGHT);
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

//            opmode.sleep(1000);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.baseBackRightMotor.setPower(speed);
        robot.baseFrontLeftMotor.setPower(speed);
        robot.baseBackLeftMotor.setPower(speed);
        robot.baseFrontRightMotor.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (robot.baseBackRightMotor.isBusy() && robot.baseFrontLeftMotor.isBusy() &&
                robot.baseBackLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy()) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.baseBackLeftMotor.setPower(speed + correction);
            robot.baseBackRightMotor.setPower(speed + correction);
            robot.baseFrontLeftMotor.setPower(speed - correction);
            robot.baseFrontRightMotor.setPower(speed - correction);

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

    public void driveSidewayLeft(double speed, double distance, float angleRobotToFace) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        int moveCounts;
        double correction = 0;

        // Four wheel Mecanum motor determine new target position, and pass to motor controller

//            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
//                    robot.baseFrontLeftMotor.getCurrentPosition(),
//                    robot.baseBackLeftMotor.getCurrentPosition(),
//                    robot.baseFrontRightMotor.getCurrentPosition(),
//                    robot.baseBackRightMotor.getCurrentPosition());
//            opmode.telemetry.update();

        moveCounts = (int) (distance * COUNTS_PER_INCH_SIDEWAY_LEFT);
        newFrontLeftTarget = robot.baseFrontLeftMotor.getCurrentPosition() - moveCounts;
        newFrontRightTarget = robot.baseFrontRightMotor.getCurrentPosition() + moveCounts;
        newBackLeftTarget = robot.baseBackLeftMotor.getCurrentPosition() + moveCounts;
        newBackRightTarget = robot.baseBackRightMotor.getCurrentPosition() - moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.baseFrontLeftMotor.setTargetPosition(newFrontLeftTarget);
        robot.baseFrontRightMotor.setTargetPosition(newFrontRightTarget);
        robot.baseBackLeftMotor.setTargetPosition(newBackLeftTarget);
        robot.baseBackRightMotor.setTargetPosition(newBackRightTarget);

        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.baseBackRightMotor.setPower(speed);
        robot.baseFrontLeftMotor.setPower(speed);
        robot.baseBackLeftMotor.setPower(speed);
        robot.baseFrontRightMotor.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (robot.baseBackRightMotor.isBusy() && robot.baseFrontLeftMotor.isBusy() &&
                robot.baseBackLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy()) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.baseBackLeftMotor.setPower(speed - correction);
            robot.baseBackRightMotor.setPower(speed - correction);
            robot.baseFrontLeftMotor.setPower(speed + correction);
            robot.baseFrontRightMotor.setPower(speed + correction);

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

    public void driveSideway(double speed, double distance, float angleRobotToFace) {
        if (distance < 0)
            driveSidewayLeft(speed, -distance, angleRobotToFace);
        else if (distance > 0)
            driveSidewayRight(speed, distance, angleRobotToFace);
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     *                 //     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void driveForward(double speed, double distance, float angleRobotToFace) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        int moveCounts;
        double correction;

        // Four wheel Mecanum motor determine new target position, and pass to motor controller
//            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
//                    robot.baseFrontLeftMotor.getCurrentPosition(),
//                    robot.baseBackLeftMotor.getCurrentPosition(),
//                    robot.baseFrontRightMotor.getCurrentPosition(),
//                    robot.baseBackRightMotor.getCurrentPosition());
        //opmode.telemetry.update();

        moveCounts = (int) (distance * COUNTS_PER_INCH_FORWARD);
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

//            opmode.telemetry.addData("Target", "L: %7d %7d R: %7d %7d",
//                    newFrontLeftTarget,
//                    newBackLeftTarget,
//                    newFrontRightTarget,
//                    newBackRightTarget);
//            opmode.telemetry.update();
//            opmode.sleep(2000);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.baseFrontLeftMotor.setPower(speed);
        robot.baseFrontRightMotor.setPower(speed);
        robot.baseBackLeftMotor.setPower(speed);
        robot.baseBackRightMotor.setPower(speed);

        while (opmode.opModeIsActive() &&
                (robot.baseFrontLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy() &&
                        robot.baseBackLeftMotor.isBusy() && robot.baseBackRightMotor.isBusy())) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

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

    public void driveBackward(double speed, double distance, float angleRobotToFace) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        int moveCounts;
        double correction;

        // Four wheel Mecanum motor determine new target position, and pass to motor controller
//            opmode.telemetry.addData("Position", "L: %7d %7d R: %7d %7d",
//                    robot.baseFrontLeftMotor.getCurrentPosition(),
//                    robot.baseBackLeftMotor.getCurrentPosition(),
//                    robot.baseFrontRightMotor.getCurrentPosition(),
//                    robot.baseBackRightMotor.getCurrentPosition());
        //opmode.telemetry.update();

        moveCounts = (int) (distance * COUNTS_PER_INCH_BACKWARD);
        newFrontLeftTarget = robot.baseFrontLeftMotor.getCurrentPosition() - moveCounts;
        newFrontRightTarget = robot.baseFrontRightMotor.getCurrentPosition() - moveCounts;
        newBackLeftTarget = robot.baseBackLeftMotor.getCurrentPosition() - moveCounts;
        newBackRightTarget = robot.baseBackRightMotor.getCurrentPosition() - moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.baseFrontLeftMotor.setTargetPosition(newFrontLeftTarget);
        robot.baseFrontRightMotor.setTargetPosition(newFrontRightTarget);
        robot.baseBackLeftMotor.setTargetPosition(newBackLeftTarget);
        robot.baseBackRightMotor.setTargetPosition(newBackRightTarget);

        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            opmode.telemetry.addData("Target", "L: %7d %7d R: %7d %7d",
//                    newFrontLeftTarget,
//                    newBackLeftTarget,
//                    newFrontRightTarget,
//                    newBackRightTarget);
//            opmode.telemetry.update();
//            opmode.sleep(2000);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.baseFrontLeftMotor.setPower(speed);
        robot.baseFrontRightMotor.setPower(speed);
        robot.baseBackLeftMotor.setPower(speed);
        robot.baseBackRightMotor.setPower(speed);

        while (opmode.opModeIsActive() &&
                (robot.baseFrontLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy() &&
                        robot.baseBackLeftMotor.isBusy() && robot.baseBackRightMotor.isBusy())) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.baseFrontLeftMotor.setPower(speed + correction);
            robot.baseFrontRightMotor.setPower(speed - correction);
            robot.baseBackLeftMotor.setPower(speed + correction);
            robot.baseBackRightMotor.setPower(speed - correction);

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

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void gyroTurn(double degrees) {

        double degreeDiff = degrees-getGlobalAngle();
        // rotate until turn is completed.
        while (Math.abs(degreeDiff) > DGREE_TOLERANCE) {
            if (Math.abs(degreeDiff) > 10) { // Fast turn
                if (degreeDiff>0) { // Left turn
                    robot.baseFrontLeftMotor.setPower(-fastTurnPower);
                    robot.baseFrontRightMotor.setPower(fastTurnPower);
                    robot.baseBackLeftMotor.setPower(-fastTurnPower);
                    robot.baseBackRightMotor.setPower(fastTurnPower);
                }
                else { // Right turn
                    robot.baseFrontLeftMotor.setPower(fastTurnPower);
                    robot.baseFrontRightMotor.setPower(-fastTurnPower);
                    robot.baseBackLeftMotor.setPower(fastTurnPower);
                    robot.baseBackRightMotor.setPower(-fastTurnPower);
                }
            }
            else { // Slow turn
                if (degreeDiff>0) { // Left turn
                    robot.baseFrontLeftMotor.setPower(-slowTurnPower);
                    robot.baseFrontRightMotor.setPower(slowTurnPower);
                    robot.baseBackLeftMotor.setPower(-slowTurnPower);
                    robot.baseBackRightMotor.setPower(slowTurnPower);
                }
                else { // Right turn
                    robot.baseFrontLeftMotor.setPower(slowTurnPower);
                    robot.baseFrontRightMotor.setPower(-slowTurnPower);
                    robot.baseBackLeftMotor.setPower(slowTurnPower);
                    robot.baseBackRightMotor.setPower(-slowTurnPower);
                }
            }

            degreeDiff = degrees-getGlobalAngle();
        }

        // turn the motors off.
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);
    }

    /**
     * Drive forward using gyro to keep the last angle.
     *
     * @param powerOrSpeed Desired speed of turn.
     * @return
     */
    public void driveForward(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.baseBackRightMotor.setPower(powerOrSpeed + correction);
        robot.baseFrontLeftMotor.setPower(powerOrSpeed - correction);
        robot.baseBackLeftMotor.setPower(powerOrSpeed - correction);
        robot.baseFrontRightMotor.setPower(powerOrSpeed + correction);
    }

    public void driveBackward(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.baseBackRightMotor.setPower(-(powerOrSpeed - correction));
        robot.baseFrontLeftMotor.setPower(-(powerOrSpeed + correction));
        robot.baseBackLeftMotor.setPower(-(powerOrSpeed + correction));
        robot.baseFrontRightMotor.setPower(-(powerOrSpeed - correction));
    }

    public void driveSidewayLeft(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.baseFrontLeftMotor.setPower(-(powerOrSpeed + correction));
        robot.baseFrontRightMotor.setPower(powerOrSpeed + correction);
        robot.baseBackLeftMotor.setPower(powerOrSpeed - correction);
        robot.baseBackRightMotor.setPower(-(powerOrSpeed - correction));
    }

    public void driveSidewayRight(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.baseFrontLeftMotor.setPower(powerOrSpeed - correction);
        robot.baseFrontRightMotor.setPower(-(powerOrSpeed - correction));
        robot.baseBackLeftMotor.setPower(-(powerOrSpeed + correction));
        robot.baseBackRightMotor.setPower(powerOrSpeed + correction);
    }

    public void stop() {
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);
    }

    /**
     * Move claw forward (power on slide servo for a specific time )
     *
     * @param clawForwardTime Desired time to slide servo.
     */
    public void moveClawForward(int clawForwardTime) {
        // Move claw forward (power on slide servo for a specific time in milliseconds,
        // tunable data. Define at the top or in config file
        robot.slideServo.setDirection(Servo.Direction.FORWARD);
        robot.slideServo.setPosition(VEX_MOTOR_RUN);
        sleep(clawForwardTime);
        robot.slideServo.setPosition(VEX_MOTOR_STOP);
    }

    public void moveClawBackward(int clawBackwardTime) {
        robot.slideServo.setDirection(Servo.Direction.REVERSE);
        robot.slideServo.setPosition(VEX_MOTOR_RUN);
        sleep(clawBackwardTime);
        robot.slideServo.setPosition(VEX_MOTOR_STOP);
    }

    public void turnClawVertical() {
        // Turn claw head 90 degree
        robot.clawheadServo.setDirection(Servo.Direction.FORWARD);
        robot.clawheadServo.setPosition(SERVO_SET_POSITION);
        sleep(300);

    }

    public void turnClawHorizontal() {
        robot.clawheadServo.setDirection(Servo.Direction.REVERSE);
        robot.clawheadServo.setPosition(SERVO_SET_POSITION);
        sleep(300);
    }

    public void closeClaw() {
        robot.clawServo.setDirection(Servo.Direction.REVERSE);
        robot.clawServo.setPosition(SERVO_SET_POSITION);
        sleep(300);
    }

    public void openClaw() {
        robot.clawServo.setDirection(Servo.Direction.FORWARD);
        robot.clawServo.setPosition(SERVO_SET_POSITION);
        sleep(300);
    }


    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void grabFoundation(int state) {
        if (state == 1) {
            robot.grabberUp = true;
            robot.foudationGrabberServo.setDirection(Servo.Direction.FORWARD);
            robot.foudationGrabberServo.setPosition(SERVO_SET_POSITION);
        }
        if (state == -1) {
            robot.grabberUp = false;
            robot.foudationGrabberServo.setDirection(Servo.Direction.REVERSE);
            robot.foudationGrabberServo.setPosition(SERVO_SET_POSITION);
        }
    }

    public void liftUp(double speed, int encoderCount) {
        int leftLiftTarget = robot.leftLiftMotor.getCurrentPosition() + encoderCount;
        int rightLiftTarget = robot.rightLiftMotor.getCurrentPosition() + encoderCount;

        leftLiftTarget = Range.clip(Math.abs(leftLiftTarget), 0, JaguarFTC1Bot.MAX_LIFT_ENCODER_VAL);
        rightLiftTarget = Range.clip(Math.abs(rightLiftTarget), 0, JaguarFTC1Bot.MAX_LIFT_ENCODER_VAL);

        robot.leftLiftMotor.setTargetPosition(leftLiftTarget);
        robot.rightLiftMotor.setTargetPosition(rightLiftTarget);

        robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        robot.leftLiftMotor.setPower(speed);
        robot.rightLiftMotor.setPower(speed);

        while (opmode.opModeIsActive() && (robot.leftLiftMotor.isBusy() && robot.rightLiftMotor.isBusy())) {
            robot.leftLiftMotor.setPower(speed);
            robot.rightLiftMotor.setPower(speed);
        }

        if (leftLiftTarget > 5 || rightLiftTarget > 5) {
            robot.leftLiftMotor.setPower(LIFT_HOLDING_POWER);
            robot.rightLiftMotor.setPower(LIFT_HOLDING_POWER);
        } else {
            robot.leftLiftMotor.setPower(0);
            robot.rightLiftMotor.setPower(0);
        }
    }

    public void liftDown(double speed, int encoderCount) {
        int leftLiftTarget = robot.leftLiftMotor.getCurrentPosition() - encoderCount;
        int rightLiftTarget = robot.rightLiftMotor.getCurrentPosition() - encoderCount;

        leftLiftTarget = Range.clip(Math.abs(leftLiftTarget), 0, JaguarFTC1Bot.MAX_LIFT_ENCODER_VAL);
        rightLiftTarget = Range.clip(Math.abs(rightLiftTarget), 0, JaguarFTC1Bot.MAX_LIFT_ENCODER_VAL);

        robot.leftLiftMotor.setTargetPosition(leftLiftTarget);
        robot.rightLiftMotor.setTargetPosition(rightLiftTarget);

        robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        robot.leftLiftMotor.setPower(speed);
        robot.rightLiftMotor.setPower(speed);

        while (opmode.opModeIsActive() && (robot.leftLiftMotor.isBusy() && robot.rightLiftMotor.isBusy())) {
            robot.leftLiftMotor.setPower(speed);
            robot.rightLiftMotor.setPower(speed);
        }

        if (leftLiftTarget > 5 || rightLiftTarget > 5) {
            robot.leftLiftMotor.setPower(LIFT_HOLDING_POWER);
            robot.rightLiftMotor.setPower(LIFT_HOLDING_POWER);
        } else {
            robot.leftLiftMotor.setPower(0);
            robot.rightLiftMotor.setPower(0);
        }
    }

    public void liftUp(double power) {
        robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (robot.leftLiftMotor.getCurrentPosition() < 30) {
            robot.leftLiftMotor.setPower(power);
            robot.rightLiftMotor.setPower(power);
        }

        robot.leftLiftMotor.setPower(LIFT_HOLDING_POWER);
        robot.rightLiftMotor.setPower(LIFT_HOLDING_POWER);
    }

    public void liftDown(double power) {
        robot.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (robot.leftLiftMotor.getCurrentPosition() > -10) {
            robot.leftLiftMotor.setPower(power);
            robot.rightLiftMotor.setPower(power);
        }

        robot.leftLiftMotor.setPower(0);
        robot.rightLiftMotor.setPower(0);
    }

    public void lightSensorDriveToObject(float angleRobotToFace) {
        int distanceCode = DISTANCE_CODE_FAR;

        while ((distanceCode != DISTANCE_CODE_CLOSE)) {
            if (distanceCode == DISTANCE_CODE_SLOWDOWN) {
                driveForward(slowdownSpeed, angleRobotToFace);
            }
            else {
                driveForward(forwardSpeed, angleRobotToFace);
            }
//            sleep(100);
            distanceCode = isSkystoneClose();
        }

        stop();
    }

    /** Check if stone is close to the robot
     *
     * @return yes or no
     */
    private int isSkystoneClose() {
        int rval = DISTANCE_CODE_CLOSE;

        double distance = robot.sensorDistance.getDistance(DistanceUnit.CM);

        if ((distance == DistanceSensor.distanceOutOfRange)
                || Double.isNaN(distance)
                || distance > 2 * STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES) {
            rval = DISTANCE_CODE_FAR;
        } else if (distance > STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES) {
            // slow down
            rval = DISTANCE_CODE_SLOWDOWN;
            //opmode.telemetry.addData("Slow down: ", distance);
        }

        return rval;
    }

}

