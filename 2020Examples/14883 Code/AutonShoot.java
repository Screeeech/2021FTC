
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="AutonShoot", group="Pushbot")

public class AutonShoot extends LinearOpMode {

    /* Declare OpMode members. */
      

    static final double     FORWARD_SPEED = 0.28;  private DcMotor leftDrive = null;
    
    private DcMotor Shot = null;
   
   
    private ElapsedTime     runtime = new ElapsedTime();

   

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
                
        Shot = hardwareMap.get(DcMotor.class, "Shot");
        
       
        waitForStart();
          Shot.setPower(1);
        
        
      sleep(1500);
        
        

        
       
    }
}
