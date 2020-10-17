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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

/** Terribly documented so I'll explain here: this just takes all out motor stuff and makes them into functions that we
 *  can later use for auton and certian pre-programmed functions in user control
 */
public class JaguarFTC1Driver {
    //Encoder counts for mecanum wheel to drive 1 inch.
    // Following data needs to be Calibrated for different environment/floor
    // The first number in comment is the calibrated value on field. The second is the one at home/hard floor.
    static final double COUNTS_PER_INCH_FORWARD = 69.9;       //69.4; //69;
    static final double COUNTS_PER_INCH_BACKWARD = 70;      //70.3; //69;
    static final double COUNTS_PER_INCH_SIDEWAY_LEFT = 76;  //75.8; //75;
    static final double COUNTS_PER_INCH_SIDEWAY_RIGHT = 76.8;   //76.8; //75;

    static public final double DGREE_TOLERANCE = 5.0;
    static public final double fastTurnPower = 0.4;
    static public final double slowTurnPower = 0.1;

    // NORTH is what the robot faces during init. To the right is EAST, to the left is WEST.
    // It won't change through out the match.
    static public final float FACING_NORTH = 0;
    static public final float FACING_SOUTH = 180;
    static public final float FACING_EAST = -90;
    static public final float FACING_WEST = 90;

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

        // Four wheel motor start motion.
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

        // four wheel Stop all motion;
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        // four wheel Turn off RUN_TO_POSITION
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

        // Four wheel motor start motion.
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

            robot.baseBackLeftMotor.setPower(speed - correction);
            robot.baseBackRightMotor.setPower(speed - correction);
            robot.baseFrontLeftMotor.setPower(speed + correction);
            robot.baseFrontRightMotor.setPower(speed + correction);

        }

        // four wheel Stop all motion;
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        // four wheel Turn off RUN_TO_POSITION
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

        // Four wheel motor start motion.
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

        // four wheel Stop all motion;
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);


        //four wheel Turn off RUN_TO_POSITION
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

        // Four wheel motor start motion.
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

        // four wheel Stop all motion;
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        // four wheel Turn off RUN_TO_POSITION
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

    public void stop(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor.ZeroPowerBehavior originalBehavior = robot.baseFrontLeftMotor.getZeroPowerBehavior();

        robot.baseFrontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        robot.baseFrontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        robot.baseBackLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        robot.baseBackRightMotor.setZeroPowerBehavior(zeroPowerBehavior);

        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        robot.baseFrontLeftMotor.setZeroPowerBehavior(originalBehavior);
        robot.baseFrontRightMotor.setZeroPowerBehavior(originalBehavior);
        robot.baseBackLeftMotor.setZeroPowerBehavior(originalBehavior);
        robot.baseBackRightMotor.setZeroPowerBehavior(originalBehavior);
    }

    // Drive back to wall by setting target position and checking if the wheels stuck
    public void driveBackwardToWall(double speed, double distance, float angleRobotToFace) {
        int lastPosition, currentPositon;
        boolean wheelNotStuck = true;

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        int moveCounts;
        double correction;

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

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.baseFrontLeftMotor.setPower(speed);
        robot.baseFrontRightMotor.setPower(speed);
        robot.baseBackLeftMotor.setPower(speed);
        robot.baseBackRightMotor.setPower(speed);

        lastPosition = robot.baseBackLeftMotor.getCurrentPosition();
        while ((robot.baseFrontLeftMotor.isBusy() && robot.baseFrontRightMotor.isBusy() &&
                        robot.baseBackLeftMotor.isBusy() && robot.baseBackRightMotor.isBusy())
                && wheelNotStuck) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.baseFrontLeftMotor.setPower(speed + correction);
            robot.baseFrontRightMotor.setPower(speed - correction);
            robot.baseBackLeftMotor.setPower(speed + correction);
            robot.baseBackRightMotor.setPower(speed - correction);

            currentPositon = robot.baseBackLeftMotor.getCurrentPosition();
            opmode.telemetry.addData("", "%d",currentPositon-lastPosition);
            opmode.telemetry.update();
            wheelNotStuck = Math.abs(currentPositon - lastPosition) > 0;
            lastPosition = currentPositon;
        }

        // four wheel Stop all motion;
        robot.baseFrontLeftMotor.setPower(0);
        robot.baseFrontRightMotor.setPower(0);
        robot.baseBackLeftMotor.setPower(0);
        robot.baseBackRightMotor.setPower(0);

        // four wheel Turn off RUN_TO_POSITION
        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

}

