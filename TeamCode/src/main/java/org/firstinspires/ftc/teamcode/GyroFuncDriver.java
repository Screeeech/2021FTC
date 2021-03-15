
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
import org.firstinspires.ftc.teamcode.Bot14787;

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
public class GyroFuncDriver {
    // Following data needs to be Calibrated for different environment/floor
    // The first number in comment is the calibrated value on field. The second is the one at home/hard floor.
    static final double COUNTS_PER_INCH_FORWARD = 69.9;       //69.4; //69; //Encoder counts for mechanum wheel to drive 1 inch.
    static final double COUNTS_PER_INCH_BACKWARD = 70;      //70.3; //69;
    static final double COUNTS_PER_INCH_SIDEWAY_LEFT = 76;  //75.8; //75;
    static final double COUNTS_PER_INCH_SIDEWAY_RIGHT = 76.8;   //76.8; //75;
    static final double STOP_DISTANCE = 6; // Stop Distance at slowdownSpeed. 13 is the Stop Distance at movingSpeed.
    static public final float HUE_THRESHOLD = 30;           //35;  //60; // Hue value for detecting Skystone image
    static final int MOTOR_STUCK_THRESHOLD = 160;

    // Following data needs to be changed for different environment/floor
    static public final double lightSensorForwardSpeed = 0.3; //0.3;  //0.2;
    static public final double lightSensorSlowdownSpeed = 0.3;//0.3;  //0.2;
    static public final double slowdownSpeed = 0.4;           //0.2; //0.1;
    static public final double movingSpeed = 0.8;             //0.8; //0.7; normal moving speed during autonomous
    static public final double parkingSpeed = 0.6;            //0.6; //0.5;
    static public final double HUE_FLOOR_THRESHOLD = 170000000;//200000000;

    static final double VEX_MOTOR_RUN = 0.7;     // The servo position to open latch
    static final double VEX_MOTOR_STOP = 0.0;
    static final double SERVO_SET_POSITION = 1.0;     // The servo set position
    static final double SERVO_INIT_POSITION = 0.0;  // The servo init position

    static public final double LIFT_POWER = 1.0;
    static final double LIFT_HOLDING_POWER = 0.2;

    // when the robot moves close to the stones (prior to pickup), the robot stops at this distance from the stones.
    static public final float STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES = 14F;
    static public final float STOP_DISTANCE_TO_FOUNDATION_LIGHTSENSOR = 13F; // In CM
    static public final float STOP_DISTANCE_TO_FOUNDATION_DISTANCESENSOR = 2F; // In Inch
    static public final int DISTANCE_CODE_CLOSE = 0;
    static public final int DISTANCE_CODE_SLOWDOWN = 1;
    static public final int DISTANCE_CODE_FAR = 2;

    static public final double DGREE_TOLERANCE = 5.0;
    static public final double fastTurnPower = 0.4;
    static public final double slowTurnPower = 0.1;

    static public final float FACING_NORTH = 0; // NORTH is what the robot faces during init. To the right is EAST, to the left is WEST. It won't change through out the match.
    static public final float FACING_SOUTH = 180;
    static public final float FACING_EAST = -90;
    static public final float FACING_WEST = 90;
    static public final int GRAB = 1;
    static public final int RELEASE = -1;
    static public final double DISTANCE_BETWEEN_ROBOT_AND_FOUNDATION = 29.0;
    static public final double DISTANCE_SLOWDOWN = 5;

    /* local OpMode members. */
    Bot14787 robot = null;
    LinearOpMode opmode = null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public GyroFuncDriver(Bot14787 arobot, LinearOpMode aopmode) {
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
        newFrontLeftTarget = robot.leftFront.getCurrentPosition() + moveCounts;
        newFrontRightTarget = robot.rightFront.getCurrentPosition() - moveCounts;
        newBackLeftTarget = robot.leftBack.getCurrentPosition() - moveCounts;
        newBackRightTarget = robot.rightBack.getCurrentPosition() + moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newFrontLeftTarget);
        robot.rightFront.setTargetPosition(newFrontRightTarget);
        robot.leftBack.setTargetPosition(newBackLeftTarget);
        robot.rightBack.setTargetPosition(newBackRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            opmode.sleep(1000);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (robot.rightBack.isBusy() && robot.leftFront.isBusy() &&
                robot.leftBack.isBusy() && robot.rightFront.isBusy()) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.leftBack.setPower(speed + correction);
            robot.rightBack.setPower(speed + correction);
            robot.leftFront.setPower(speed - correction);
            robot.rightFront.setPower(speed - correction);

        }

        // Dylan - four wheel Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Dylan - four wheel Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        newFrontLeftTarget = robot.leftFront.getCurrentPosition() - moveCounts;
        newFrontRightTarget = robot.rightFront.getCurrentPosition() + moveCounts;
        newBackLeftTarget = robot.leftBack.getCurrentPosition() + moveCounts;
        newBackRightTarget = robot.rightBack.getCurrentPosition() - moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newFrontLeftTarget);
        robot.rightFront.setTargetPosition(newFrontRightTarget);
        robot.leftBack.setTargetPosition(newBackLeftTarget);
        robot.rightBack.setTargetPosition(newBackRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.rightBack.setPower(speed);
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (robot.rightBack.isBusy() && robot.leftFront.isBusy() &&
                robot.leftBack.isBusy() && robot.rightFront.isBusy()) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.leftBack.setPower(speed - correction);
            robot.rightBack.setPower(speed - correction);
            robot.leftFront.setPower(speed + correction);
            robot.rightFront.setPower(speed + correction);

        }

        // Dylan - four wheel Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Dylan - four wheel Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        newFrontLeftTarget = robot.leftFront.getCurrentPosition() + moveCounts;
        newFrontRightTarget = robot.rightFront.getCurrentPosition() + moveCounts;
        newBackLeftTarget = robot.leftBack.getCurrentPosition() + moveCounts;
        newBackRightTarget = robot.rightBack.getCurrentPosition() + moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newFrontLeftTarget);
        robot.rightFront.setTargetPosition(newFrontRightTarget);
        robot.leftBack.setTargetPosition(newBackLeftTarget);
        robot.rightBack.setTargetPosition(newBackRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            opmode.telemetry.addData("Target", "L: %7d %7d R: %7d %7d",
//                    newFrontLeftTarget,
//                    newBackLeftTarget,
//                    newFrontRightTarget,
//                    newBackRightTarget);
//            opmode.telemetry.update();
//            opmode.sleep(2000);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.leftFront.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightBack.setPower(speed);

        while (opmode.opModeIsActive() &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy() &&
                        robot.leftBack.isBusy() && robot.rightBack.isBusy())) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.leftFront.setPower(speed - correction);
            robot.rightFront.setPower(speed + correction);
            robot.leftBack.setPower(speed - correction);
            robot.rightBack.setPower(speed + correction);

        }

        // Dylan - four wheel Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);


        // Dylan - four wheel Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        newFrontLeftTarget = robot.leftFront.getCurrentPosition() - moveCounts;
        newFrontRightTarget = robot.rightFront.getCurrentPosition() - moveCounts;
        newBackLeftTarget = robot.leftBack.getCurrentPosition() - moveCounts;
        newBackRightTarget = robot.rightBack.getCurrentPosition() - moveCounts;

        // Four wheel Mecanum motor Set Target and Turn On RUN_TO_POSITION
        robot.leftFront.setTargetPosition(newFrontLeftTarget);
        robot.rightFront.setTargetPosition(newFrontRightTarget);
        robot.leftBack.setTargetPosition(newBackLeftTarget);
        robot.rightBack.setTargetPosition(newBackRightTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            opmode.telemetry.addData("Target", "L: %7d %7d R: %7d %7d",
//                    newFrontLeftTarget,
//                    newBackLeftTarget,
//                    newFrontRightTarget,
//                    newBackRightTarget);
//            opmode.telemetry.update();
//            opmode.sleep(2000);

        // Dylan - Four wheel motor start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.leftFront.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightBack.setPower(speed);

        while (opmode.opModeIsActive() &&
                (robot.leftFront.isBusy() && robot.rightFront.isBusy() &&
                        robot.leftBack.isBusy() && robot.rightBack.isBusy())) {

            // Use gyro to drive in a straight line.
            correction = checkDirection(angleRobotToFace);

            //opmode.telemetry.addData("", "Angle %7.2f Correction %7.2f",globalAngle,correction);
            //opmode.telemetry.update();

            robot.leftFront.setPower(speed + correction);
            robot.rightFront.setPower(speed - correction);
            robot.leftBack.setPower(speed + correction);
            robot.rightBack.setPower(speed - correction);

        }

        // Dylan - four wheel Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // Dylan - four wheel Turn off RUN_TO_POSITION
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                    robot.leftFront.setPower(-fastTurnPower);
                    robot.rightFront.setPower(fastTurnPower);
                    robot.leftBack.setPower(-fastTurnPower);
                    robot.rightBack.setPower(fastTurnPower);
                }
                else { // Right turn
                    robot.leftFront.setPower(fastTurnPower);
                    robot.rightFront.setPower(-fastTurnPower);
                    robot.leftBack.setPower(fastTurnPower);
                    robot.rightBack.setPower(-fastTurnPower);
                }
            }
            else { // Slow turn
                if (degreeDiff>0) { // Left turn
                    robot.leftFront.setPower(-slowTurnPower);
                    robot.rightFront.setPower(slowTurnPower);
                    robot.leftBack.setPower(-slowTurnPower);
                    robot.rightBack.setPower(slowTurnPower);
                }
                else { // Right turn
                    robot.leftFront.setPower(slowTurnPower);
                    robot.rightFront.setPower(-slowTurnPower);
                    robot.leftBack.setPower(slowTurnPower);
                    robot.rightBack.setPower(-slowTurnPower);
                }
            }

            degreeDiff = degrees-getGlobalAngle();
        }

        // turn the motors off.
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
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
        robot.rightBack.setPower(powerOrSpeed + correction);
        robot.leftFront.setPower(powerOrSpeed - correction);
        robot.leftBack.setPower(powerOrSpeed - correction);
        robot.rightFront.setPower(powerOrSpeed + correction);
    }

    public void driveBackward(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.rightBack.setPower(-(powerOrSpeed - correction));
        robot.leftFront.setPower(-(powerOrSpeed + correction));
        robot.leftBack.setPower(-(powerOrSpeed + correction));
        robot.rightFront.setPower(-(powerOrSpeed - correction));
    }

    public void driveSidewayLeft(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.leftFront.setPower(-(powerOrSpeed + correction));
        robot.rightFront.setPower(powerOrSpeed + correction);
        robot.leftBack.setPower(powerOrSpeed - correction);
        robot.rightBack.setPower(-(powerOrSpeed - correction));
    }

    public void driveSidewayRight(double powerOrSpeed, float angleRobotToFace) {
        double correction = checkDirection(angleRobotToFace);
        powerOrSpeed = Range.clip(Math.abs(powerOrSpeed), 0.0, 1.0);
        robot.leftFront.setPower(powerOrSpeed - correction);
        robot.rightFront.setPower(-(powerOrSpeed - correction));
        robot.leftBack.setPower(-(powerOrSpeed + correction));
        robot.rightBack.setPower(powerOrSpeed + correction);
    }

    public void stop() {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void stop(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        DcMotor.ZeroPowerBehavior originalBehavior = robot.leftFront.getZeroPowerBehavior();

        robot.leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        robot.rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        robot.leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        robot.rightBack.setZeroPowerBehavior(zeroPowerBehavior);

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setZeroPowerBehavior(originalBehavior);
        robot.rightFront.setZeroPowerBehavior(originalBehavior);
        robot.leftBack.setZeroPowerBehavior(originalBehavior);
        robot.rightBack.setZeroPowerBehavior(originalBehavior);
    }




    public void flickRing(int flickRingTime) {
        // Move claw forward (power on slide servo for a specific time in milliseconds,
        // tunable data. Define at the top or in config file
        robot.flickerServo.setDirection(Servo.Direction.FORWARD);
        robot.flickerServo.setPosition(-0.5);
        sleep(flickRingTime);
        robot.flickerServo.setPosition(0.5);
    }

    public void shootRing(int shootRingTime) {
        robot.leftShooter.setPower(0.75);
        robot.rightShooter.setPower(0.75);
        sleep(shootRingTime);
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
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