package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "RingTensorFlow", group = "Concept")
//@Disabled
public class RingTensorFlow extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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
            "AZMENAr/////AAABmZDmDvMpFk+QrHyz4b7F4D+CdBmV4gePfNAcvbsGSozTbqiXRwNX/HWW1kwv0Gc7wwbz/1qR4SxNXO/nmrxq9IFPgTJU2jxMFfWI1EKWSwSdp/AjEVmzzhNFiHHkQlx+pIFVSzxSNgdovPv/X7cRtAt+dkNKIcJftrBOUUabrHlhB5/nYAgqvDU7NprhqOw18J1syurAcDRuLfR8WzygDsRWkWDkSNhyR6lGFTAiFj9u8Sfcu3xHmF3ZaddBFOAwY4HJd35qKtX3XO0hbTTj0DeGeakeYH34jJiBxGYCjfw8mzh1RVQssjvWq+5Z2ZcOagznpkrQ0qv5TTfjAwZcBQK0MrwwSS8Oi1gDJVo7WSs1";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    Bot14787 robot = new Bot14787();
    GyroFuncDriver gyroDriver = new GyroFuncDriver(robot, this);


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                gyroDriver.driveForward(0.8, 35, 0); // Move forward to the starter stack

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                gyroDriver.driveForward(0.8, 50, 0); // Move forward to the A target zone
                                sleep(50); // stop for 50 seconds
                                gyroDriver.gyroTurn(38); // turn to a specific angle in which it moves forward after
                                gyroDriver.driveForward(0.8, 10, -45); // will move forward to push the wobble to the target zone
                                sleep(50); // stop for 50 seconds
                                gyroDriver.driveBackward(0.8, 10, -45); // move back to the area from where it turned after it finished pushing to the wobble
                                sleep(50); // stop for 50 seconds
                                gyroDriver.gyroTurn(-38); // turn to the degree of where the shooter is facing as the front
                                gyroDriver.driveSidewayLeft(0.8, 34, 0); // it will move right actually and shoot on the other side
                                sleep(50); // stop for 50 seconds
                                gyroDriver.flickRing(4); // the servo will try to flick it for 4 seconds and this time will have to be reduced
                                gyroDriver.shootRing(4);// this will shoot the two rings but this still has to be tested
                                sleep(50); // stop for 50 seconds
                                gyroDriver.driveBackward(0.8, 28, 0); // park


                            }
                            else if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {

                            }
                            else {

                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
