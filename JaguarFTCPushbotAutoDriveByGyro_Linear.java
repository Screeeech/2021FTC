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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class JaguarFTCPushbotAutoDriveByGyro_Linear extends LinearOpMode {

    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);

    /* Declare OpMode members. */

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {
        robot.robotInit(hardwareMap, telemetry);

        robot.baseFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.baseBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %5.2f",robot.lastAngles.firstAngle);
            telemetry.addData(">", "globalAngle = %5.2f",driver.getGlobalAngle());
            telemetry.addData(">", "correction = %5.2f",driver.checkDirection());
            telemetry.update();
        }

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        waitForStart();

        // restart gyro movement tracking.
        robot.resetAngle(); // Reset global angle
        telemetry.addData(">", "Angle Reset");
        telemetry.update();

        // test gyro data reading without robot driving
/*
        while ( isStarted()) {
            telemetry.addData(">", "globalAngle = %5.2f",driver.getGlobalAngle());
            telemetry.addData(">", "correction = %5.2f",driver.checkDirection());
            telemetry.update();
        }
*/
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        //telemetry.addData("Prepare to drive FWD", "");
        //telemetry.update();
        //sleep(1000);
        //driver.gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        telemetry.addData("Prepare to drive Sideway to the Right", "");
        telemetry.update();
        sleep(1000);
        driver.gyroDriveSidewayRight(DRIVE_SPEED, 48.0);    // Drive REV 48 inches
        sleep(10000);
        // gyroTurn(TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        //gyroHold(TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        //gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        //gyroTurn(TURN_SPEED, 45.0);         // Turn  CW  to  45 Degrees
        //gyroHold(TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        //gyroTurn(TURN_SPEED, 0.0);         // Turn  CW  to   0 Degrees
        //gyroHold(TURN_SPEED, 0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        //telemetry.addData("Prepare to drive REV", "");
        //telemetry.update();
        //sleep(1000);
        //driver.gyroDrive(DRIVE_SPEED, -48.0, 0.0);    // Drive REV 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}