package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="FoundationRed", group="Pushbot")

public class FoundationRed extends LinearOpMode {

    /* Declare OpMode members. */
      

    static final double     FORWARD_SPEED = 0.4;  
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo linearServo = null;
   private DcMotor linearmotor = null;
    //Use a Pushbot's hardware
   


    @Override
    public void runOpMode() {

                
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
          linearServo = hardwareMap.get(Servo.class, "linearServo");
         linearmotor = hardwareMap.get(DcMotor.class, "linearmotor");
       
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
      

        /*while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        */
          leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        //this is drive forward
      sleep(750);

        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        sleep(650);
        //strafe right
        
        leftFront.setPower(-FORWARD_SPEED + 0.05);
        rightFront.setPower(FORWARD_SPEED - 0.05);
        leftBack.setPower(-FORWARD_SPEED + 0.05);
        rightBack.setPower(FORWARD_SPEED - 0.05);
        
      sleep(0);
        leftServo.setPosition(0.45);
        rightServo.setPosition(.96);
        sleep(1300);
        //drop servos
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        //reverse with foundation
      sleep(2700);
     
        leftServo.setPosition(-0.96);
        rightServo.setPosition(-0.65);
        sleep(1550);
        //pick up servos
         leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        //this is drive forward
      sleep(100);
        
        leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
      sleep(2420);
      //strafe to park
     
     
      
    }
}