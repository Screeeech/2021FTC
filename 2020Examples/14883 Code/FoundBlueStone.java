package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="FoundBlueStone", group="Pushbot")

public class FoundBlueStone extends LinearOpMode {

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
    // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();



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
      sleep(650);

        leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        sleep(570);
        //strafe right
         leftFront.setPower(-FORWARD_SPEED + 0.05);
        rightFront.setPower(FORWARD_SPEED - 0.05);
        leftBack.setPower(-FORWARD_SPEED + 0.05);
        rightBack.setPower(FORWARD_SPEED - 0.05);
        
      sleep(0);
        
        
        leftServo.setPosition(0.65);
        rightServo.setPosition(0.6);
        sleep(1400);
        //drop servos
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        //reverse with foundation
      sleep(3330);
      
        leftServo.setPosition(-0.96);
        rightServo.setPosition(-0.65);
        sleep(1550);
        //pick up servos from found
         leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        //this is drive forward to space
      sleep(150);
        
         leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
      sleep(1650);
      //strafe to bridge
        
        
         leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        //this is drive forwardto high
      sleep(670);
        
        
      
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
      sleep(1760);
      //strafe to quarry stone
      
       leftFront.setPower(-FORWARD_SPEED + .15);
        rightFront.setPower(FORWARD_SPEED - .15);
        leftBack.setPower(-FORWARD_SPEED + .15);
        rightBack.setPower(FORWARD_SPEED - .15);
        //this is drive forward to stone
      sleep(150);
     
    
     
      leftServo.setPosition(0.17);
    rightServo.setPosition(-.78);
    sleep(2400);
 //servo stone
     
     
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        //reverse with stone
      sleep(1950);
      
        leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
      sleep(1700);
        
      
     leftServo.setPosition(-0.96);
        rightServo.setPosition(0.65);
        sleep(1600);
        
      leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
      sleep(1100);
        
    linearmotor.setPower(-FORWARD_SPEED);
     sleep(1050);
     
     
    }
}
