
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="IntakeTest", group="Pushbot")

public class IntakeTest extends LinearOpMode {

    /* Declare OpMode members. */
      

    static final double     FORWARD_SPEED = 1;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
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
       
       
        waitForStart();
          leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
      
      sleep(1500);
        
        

        
       
    }
}
