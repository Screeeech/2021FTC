package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="FoundRedStone", group="Pushbot")

public class FoundRedStone extends LinearOpMode {

    /* Declare OpMode members. */
      

    static final double     FORWARD_SPEED = 0.4;  
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo linearServo = null;
    private Servo ServoC;
   private DcMotor linearmotor = null;
    //Use a Pushbot's hardware
 
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
        ServoC = hardwareMap.get(Servo.class, "ServoC");
       
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
        leftServo.setPosition(0.65);
        rightServo.setPosition(-0.65);
        sleep(1300);
        //drop servos
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        //reverse with foundation
      sleep(2700);
     
        leftServo.setPosition(-0.96);
        rightServo.setPosition(0.65);
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
      sleep(3720);
      //strafe to park
     
        
       leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        //this is drive forward to sotne
      sleep(850);
     
     
        timer.reset();
while (timer.seconds() < .4) {
                telemetry.addLine("Waiting...");
                telemetry.update();
}
     
      leftServo.setPosition(0.17);
    rightServo.setPosition(-.78);
    sleep(350);
 
     
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
        //reverse with foundation
      sleep(1200);
   
    
      


       leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
      sleep(1700);
     

  timer.reset();
while (timer.seconds() < 1.7) {
                telemetry.addLine("Waiting...");
                telemetry.update();
}
        
        leftServo.setPosition(-0.96);
        rightServo.setPosition(0.65);
        sleep(2900);
        
        
    
       leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(-FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
      sleep(450);
     
      
    
       leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(FORWARD_SPEED);
        rightBack.setPower(-FORWARD_SPEED);
      sleep(1700);
     
     linearmotor.setPower(-FORWARD_SPEED);
     sleep(1050);
     
    }
}
