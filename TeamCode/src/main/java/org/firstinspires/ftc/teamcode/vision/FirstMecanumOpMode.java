package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Op Mode", group="PiRhos")

public class FirstMecanumOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;


   @Override
   public void runOpMode(){
       telemetry.addData("Status", "Initialized");
       telemetry.update();

       // Initialize the hardware variables. Note that the strings used here as parameters
       // to 'get' must correspond to the names assigned during the robot configuration
       // step (using the FTC Robot Controller app on the phone).
       leftFront = hardwareMap.get(DcMotor.class, "left_front");
       rightFront = hardwareMap.get(DcMotor.class, "right_front");
       leftRear = hardwareMap.get(DcMotor.class, "left_back");
       rightRear = hardwareMap.get(DcMotor.class, "right_back");

       // Most robots need the motor on one side to be reversed to drive forward
       // Reverse the motor that runs backwards when connected directly to the battery
       leftFront.setDirection(DcMotor.Direction.REVERSE);
       leftRear.setDirection((DcMotor.Direction.FORWARD));
       rightFront.setDirection(DcMotor.Direction.FORWARD);
       rightRear.setDirection(DcMotor.Direction.FORWARD);

       // Wait for the game to start (driver presses PLAY)
       waitForStart();
       runtime.reset();

       while (opModeIsActive()){
       double LFPower;
       double LBPower;
       double RFPower;
       double RBPower;

       double y = -gamepad1.left_stick_y;
       double x = gamepad1.left_stick_x;
       double rx = gamepad1.right_stick_x;

       LFPower = (y + x + rx);
       LBPower = (y - x + rx);
       RFPower = (y - x - rx);
       RBPower = (y + x - rx);

           if (Math.abs(LFPower) > 1 || Math.abs(LBPower) > 1 ||
                   Math.abs(RFPower) > 1 || Math.abs(RBPower) > 1 ) {
               // Find the largest power
               double max = 0;
               max = Math.max(Math.abs(LFPower), Math.abs(LBPower));
               max = Math.max(Math.abs(max), Math.abs(RFPower));
               max = Math.max(Math.abs(max),Math.abs(RBPower) );

               // Divide everything by max (it's positive so we don't need to worry
               // about signs)
               LFPower /= max;
               LBPower /= max;
               RFPower /= max;
               RBPower /= max;
           }

           leftFront.setPower(LFPower);
           leftRear.setPower(LBPower);
           rightFront.setPower(RFPower);
           rightRear.setPower(RBPower);





       }



   }

}
