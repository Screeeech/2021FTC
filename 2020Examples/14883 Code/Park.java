
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Park", group="Pushbot")

public class Park extends LinearOpMode {

    /* Declare OpMode members. */
      

    static final double     FORWARD_SPEED = 0.28;  private DcMotor leftDrive = null;
    
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    private Servo linearServo = null;
    private DcMotor linearmotor = null;
   private Servo ServoC;
    //Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

   

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
                
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        linearServo = hardwareMap.get(Servo.class, "linearServor");
         linearmotor = hardwareMap.get(DcMotor.class, "linearmotor");
        ServoC = hardwareMap.get(Servo.class, "ServoC");
       
        waitForStart();
          leftFront.setPower(-FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftBack.setPower(-FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);
        
      sleep(1500);
        
        

        
       
    }
}
