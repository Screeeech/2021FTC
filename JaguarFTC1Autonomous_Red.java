/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "JaguarFTC1Autonomous_Red", group = "Concept")
//@Disabled
public class JaguarFTC1Autonomous_Red extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    // for now we only define two directions as we only need to move left or right to find a skystone
    // later we may need to have four directions (i.e., N/S/W/E).
    private static final int DIRECTION_LEFT = 0;
    private static final int DIRECTION_RIGHT = 1;

    // the following constants need to be adjusted accordingly

    // when the robot moves close to the stones (prior to pickup), the robot stops at this distance from the stones.
    static public final float STOP_DISTANCE_BETWEEN_ROBOT_AND_STONES = 30F;
    static public final int DISTANCE_CODE_CLOSE = 0;
    static public final int DISTANCE_CODE_SLOWDOWN = 1;
    static public final int DISTANCE_CODE_FAR = 2;

    // we use skystone x coordinate in the image to determine alignment. we consider "x +/- ALIGNMENT_TOLERANCE" to be aligned.
    static public final int ALIGNMENT_TOLERANCE = 800;
    // Need to fine tune. The number used to calculate how far the robot center away from the skystone center.
    // Obtained from the size of camera view angle and geometry compute
    static public final float ALIGN_DISTANCE_CONSTANT = 11;
    static public final int OFFSET_THRESHHOLD = 5;
    static public final int STONE_WIDTH = 8;

    private double forwardSpeed = 0.2;
    private double adjustSpeed = 0.05;
    private double slowdownSpeed = 0.1;
    private double movingSpeed = 0.7; // normal moving speed during autonomous
    private int clawForwardTime = 600; // milliseconds. The time to move slide forward, which control the claw forward distance.
    private double backupDistance = 12.0; // The distance to move the robot backward after grabbing the stone
    private double distanceToOtherSide = 80.0; // The distance to move to the other side of the field during autonomous after grabbing the stone

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ARpLS2D/////AAABmUSs+uDmaEVmqYp/+8UETFYRiXYhl5m+LgYAak4XdC+m3wYTd6/lGLVyBxMZm0KvDOqR/jB+BNRJAF+lZPvJL4r8obZXaOVJkDQ40rWpgtvV6UKVXO27b8pHOdQsmH5pkzXkl62l3ZFPculyuRZzKKniYEoXMELwwQUBEBBfH88fNLHgABQn5kwpb96SnQmNrkwKTnQ62rANGteDmymP+gsiWGtq5ZqAs8CnhHAF9S64lS8scYHixgRCVhyrdQ5G9YoibLTOM8QJGmVJcfB4EaLDLYItVNPJZDzJ0Ssdr613dEfHkWoLG2zCZ9qmZQmHaSkRS3vFtvzberCkAIC0tLXQUy9vEah6OxQlHqW0YVqO";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    JaguarFTC1Bot         robot   = new JaguarFTC1Bot();
    JaguarFTC1Driver    driver = new JaguarFTC1Driver(robot,this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.robotInit(hardwareMap, telemetry);
        telemetry.addData("Wait for Autonomous Ready", "");
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        sleep(3000); // wait for TF finish init

        // now we do not clear for debugging. this should be enabled after we finish testing
        //telemetry.setAutoClear(false);

        Recognition skystone = null;
        int loopCount = 5;
        int count = 0;
        float skystoneCenter=0;
        float skystoneWidth=0;
        float skystoneOffset=0;

        // Phase 1: We loop multiple times to make sure we find the Skystone
        while (loopCount > 0) {
            skystone = getSkystoneInSight(true);
            if (skystone != null) {
                count++;
                skystoneCenter += (skystone.getLeft() + skystone.getRight()) / 2;
                skystoneWidth += skystone.getWidth();
            }
            loopCount--;
            sleep(200);
        }

        if (count>0) {
            skystoneCenter = skystoneCenter/count;
            skystoneWidth = skystoneWidth/count;
            telemetry.addData("Found Skystone", "%d times, C: %7.2f, W: %7.2f", count, skystoneCenter, skystoneWidth);
            skystoneOffset = ALIGN_DISTANCE_CONSTANT*(skystoneCenter-320)/skystoneWidth;
            telemetry.addData("", "Skystone Offset: %7.2f", skystoneOffset);
        }
        else {
            telemetry.addData("No Skystone", "");
        }
        telemetry.addData("Autonomous Ready", "");
        telemetry.update();

        /** Wait for the game to begin */
        waitForStart();


        // Phase 2: Align robot with skystone
        // move robot sideway based on skystoneCenter value. Move to the left if < 320. Move to the right if > 320

        // Assume the robot is perfectly aligned with the center of the stone during setup,
        // then the skystone must be offset either 8, -8, or 0 inches from the center of robot.
        //telemetry.addData("", "Skystone Offset: %7.2f", skystoneOffset);
        if (skystoneOffset>OFFSET_THRESHHOLD)
            skystoneOffset=STONE_WIDTH;
        else if (skystoneOffset<-OFFSET_THRESHHOLD)
            skystoneOffset=-STONE_WIDTH;
        else
            skystoneOffset=0;
        driver.driveSideway(0.3, skystoneOffset);

        // Phase 3: Move robot close to skystone
        telemetry.addData("", "Move robot close to skystone");
        telemetry.update();

        int distanceCode = DISTANCE_CODE_FAR;

        while ((distanceCode != DISTANCE_CODE_CLOSE) && opModeIsActive()) {
            if (distanceCode == DISTANCE_CODE_SLOWDOWN) {
                driver.driveForward(slowdownSpeed);
            }
            else {
                driver.driveForward(forwardSpeed);
            }
            sleep(100);
            distanceCode = isSkystoneClose();
        }

        driver.stop();


        // Step 5: Claw pick up stone
        clawPickupStone(clawForwardTime);
        sleep(100);

        // Step 6: Moving back by gyro driver backward
        driver.driveBackward(movingSpeed, backupDistance);
        sleep(100);

        // Step 7: Moving sideway to other side of the field
        driver.driveSidewayRight(movingSpeed, distanceToOtherSide);
        sleep(100);

        // Step 10: Open claw to drop the stone
        clawDropStone(0);

        // TODO: Step 11: Going back to pick up another Skystone or a Stone if another Skystone is the first one
        // drive back
        double distanceToSecondSkystone = distanceToOtherSide + 24.0;
        driver.driveSidewayLeft(movingSpeed, distanceToSecondSkystone);
        driver.driveForward(movingSpeed, backupDistance);


        // repeat the same steps as the first skystone
        clawPickupStone(clawForwardTime);
        sleep(100);

        // Step 6: Moving back by gyro driver backward
        driver.driveBackward(movingSpeed, backupDistance);
        sleep(100);

        // Step 7: Moving sideway to other side of the field
        driver.driveSidewayRight(movingSpeed, distanceToOtherSide);
        sleep(100);

        // Step 10: Open claw to drop the stone
        clawDropStone(0);

        // parking
        driver.driveSidewayLeft(movingSpeed, distanceToSecondSkystone/2);


        // TODO: Last Step: Move away from the delivered stone (robot can not touch the stone) or Park robot

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the Vuforia localization engine.
     */

   // private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        /*
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    } */

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);

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
            telemetry.addData("Slow down: ", distance);
        }

        return rval;
    }


    /**
     * Get a skystone in the current sight. If there are two skystones, then get the one on the right
     *
     */
    private Recognition getSkystoneInSight (boolean updated) {
        Recognition skystone = null;

        // get all the objects detected in the current view
        List<Recognition> recognitions = null;
        if (updated) {
            recognitions = tfod.getUpdatedRecognitions();
        }
        else {
            recognitions = tfod.getRecognitions();
        }

        if (recognitions != null) {
            // step through the list of recognitions and find the rightmost one.
            for (Recognition recognition : recognitions) {
                if (recognition.getLabel().equals(LABEL_SKYSTONE)) {
                    skystone = recognition;
                    break;
                }
            }
        }

        return skystone;
    }


    /**
     * Find a skystone in a given direction at a given speed
     *
     * @param
     * @return
     */
    private Recognition findSkystone (int direction, double speed) {
        Recognition skystone = getSkystoneInSight(false);

//        telemetry.addData("find skystone: ", skystone);
//        telemetry.update();

        skystone = getSkystoneInSight(true);

        while (skystone == null && opModeIsActive()) {
            if (direction == DIRECTION_LEFT) {
                driver.driveSidewayLeft(speed);
            } else if (direction == DIRECTION_RIGHT) {
                driver.driveSidewayRight(speed);
            }

            //sleep(100);
            skystone = getSkystoneInSight(true);
        }


        // stop robot
        driver.stop();

        if(skystone == null)
        {
            // This should not happen, unless wrong direction is given
            telemetry.addData("Something is wrong. Check direction:", direction);
            telemetry.update ();
        }

        return skystone;
    }

    /**
     * This method is used to adjust the robot's position to the (approximate) center of the skystone.
     * It is supposed to be called after the robot detects a skystone and stops.
     */
    private void align () {
        Recognition skystone = getSkystoneInSight(false);

        int aligned = checkAlignment(skystone);
        while (skystone != null && aligned != 0) {
            if (aligned == 1) {
                // the robot needs to move left
                driver.driveSidewayLeft(adjustSpeed);
                sleep(100);
                skystone = getSkystoneInSight(false);
                if (skystone == null) {
                    skystone = findSkystone(DIRECTION_LEFT, adjustSpeed);
                }
            }
            else {
                driver.driveSidewayRight(adjustSpeed);

                sleep(100);
                skystone = getSkystoneInSight(false);
                if (skystone == null) {
                    skystone = findSkystone(DIRECTION_RIGHT, adjustSpeed);
                }

            }

            aligned = checkAlignment(skystone);
        }

        // stop robot
        driver.stop ();

        if (skystone != null) {
            telemetry.addData("skystone centered (left): ", getBoundingBoxLeft(skystone));
            telemetry.addData("skystone centered (object width): ", getBoundingBoxWidth(skystone));
        }
        else {
            telemetry.addData ("inside align: something is wrong.", aligned);
        }

        telemetry.update();
    }



    /**
     * This method return the left value of the bounding box if the robot is perfectly aligned
     * with the skystone.
     *
     * @param skystone
     * @return
     */
    private float getPerfectLeft (Recognition skystone) {
        float left = getBoundingBoxLeft(skystone);
        float objectWidth = getBoundingBoxWidth(skystone);
        float imageWidth = getImageWidth(skystone);

        return imageWidth / 2 + objectWidth / 2;
    }


    /**
     * This method checks the alignment of the robot with the skystone
     *
     * @param skystone
     * @return  0: robot is aligned with skystone;
     *          1: robot is to the left of skystone;
     *          -1: if robot is to the right of skystone
     */
    private int checkAlignment (Recognition skystone) {
        int aligned = 0;


        // if skystone's left is equal to perfectLeft, then skystone is right in the middle
        // and thus we consider skystone and robot to be perfectly aligned
        float perfectLeft = getPerfectLeft (skystone);
        float actualLeft = getBoundingBoxLeft (skystone);

        // if the difference between actualLeft and perfectLeft is less than a predefined threshold
        // we consider skystone and robot to be aligned (even though not perfectly aligned).
        if ((Math.abs(actualLeft - perfectLeft) > ALIGNMENT_TOLERANCE)) {
            if (actualLeft - perfectLeft > ALIGNMENT_TOLERANCE) {
                // robot is to the right of skystone
                aligned = 1;
            }
            else {
                // robot is to the left of skystone
                aligned = -1;
            }

        }

        telemetry.addData("checkAlignment:perfectLeft: ", perfectLeft);
        telemetry.addData("checkAlignment:actualLeft: ", actualLeft);
        telemetry.addData("checkAlignment:aligned: ", aligned);
        telemetry.update ();

        return aligned;

    }


    // The following methods are needed to transpose coordinates due to the portrait mode
    private float getBoundingBoxLeft (Recognition skystone) {
        return skystone.getBottom();
    }


    private float getBoundingBoxWidth (Recognition skystone) {
        return skystone.getHeight ();
    }

    private float getImageWidth (Recognition skystone) {
        return skystone.getImageHeight ();
    }

    private void clawPickupStone(int clawForwardTime) {

        driver.moveClawForward(clawForwardTime);
        sleep(300);

        // Turn claw head 90 degree
        driver.turnClawVertical();
        sleep(300);

        // Grab Skystone
        driver.closeClaw();
        sleep(300);

        // Move claw back a little bit
        driver.moveClawBackward(100);
    }

    public void clawDropStone(int clawForwardTime) {

        // Open claw
        driver.openClaw();

        /* Following action may not be needed. After autonomous, we want to keep the claw at the current position for manual control
        // Turn claw head 90 degree
        robot.clawheadServo.setDirection(Servo.Direction.REVERSE);
        robot.clawheadServo.setPosition(SERVO_SET_POSITION);

        // Move claw backward (power on slide servo for a specific time in milliseconds,
        // tunable data. Define at the top or in config file
        robot.slideServo.setDirection(Servo.Direction.REVERSE);
        robot.slideServo.setPosition(SERVO_SET_POSITION);
        sleep(clawForwardTime);

         */
    }
}
