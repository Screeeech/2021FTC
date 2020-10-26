package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpPossible", group = "Linear Opmode")
public class TeleOpPossible extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor inLeft;
    private DcMotor inRight;
    private DcMotor inLeft2;
    private DcMotor inRight2;
    
    


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        inRight = hardwareMap.get(DcMotor.class, "inRight");
        inLeft = hardwareMap.get(DcMotor.class, "inLeft");
        inRight2 = hardwareMap.get(DcMotor.class, "inRight2");
        inLeft2 = hardwareMap.get(DcMotor.class, "inLeft2");
        

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        inRight.setDirection(DcMotor.Direction.FORWARD);
        inLeft.setDirection(DcMotor.Direction.FORWARD);
        inRight2.setDirection(DcMotor.Direction.FORWARD);
        inLeft2.setDirection(DcMotor.Direction.FORWARD);
        
        
       
        waitForStart();

        while (opModeIsActive()) {

            double leftStickY = -gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;
            mecDrive(leftStickY,rightStickX,leftStickX);
            
    while (gamepad1.x) {
    inRight.setPower(1);
    inLeft.setPower(-1);
    inRight2.setPower(-1);
    inLeft2.setPower(1);
    telemetry.addData("Gamepad1", "X Pressed");
    
     if (gamepad1.x) {
    inRight.setPower(0);
    inLeft.setPower(0);
    inLeft2.setPower(0);
    inRight2.setPower(0);
    telemetry.addData("Gamepad1", "X Pressed");
    

    }

    
}
 
}

    } 
    public void mecDrive(double forward, double turn, double strafe) {
     leftFront.setPower(forward + turn - strafe);
     rightFront.setPower(forward - turn - strafe);
     leftBack.setPower(forward + turn + strafe);
     rightBack.setPower(forward - turn + strafe);
}

}