package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Encoder Test Op Mode", group="PiRhos")
@Disabled
public class EncoderTest extends LinearOpMode {
    public DcMotor FW = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        FW = hardwareMap.get(DcMotor.class, "shooter_wheel");
        FW.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        double flywheelPower = -0.4;

        while (opModeIsActive()){
            if (gamepad1.right_bumper) {
                flywheelPower += .1;

            } else if (gamepad1.left_bumper) {
                flywheelPower -= .1;
            }

            if(flywheelPower>1){
                flywheelPower = 1;
            }
            if(flywheelPower<-1){
                flywheelPower = -1;
            }

            FW.setPower(flywheelPower);
            sleep(50);
            double firstPos = FW.getCurrentPosition();
            sleep(100);
            double newPos = FW.getCurrentPosition();

            double deltaPos = newPos - firstPos;


            telemetry.addData("Status", "Flywheel Power: " + flywheelPower);
            telemetry.addData("Status", "Encoder Change: " + deltaPos);
            telemetry.addData("Status", "Encoder start: " + firstPos);
            telemetry.addData("Status", "Encoder new: " + newPos);




            telemetry.update();


        }
    }
    public void getEncoder(){
        double firstPos = FW.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds()<1000)
        {
            sleep(100);
        }
        double newPos = FW.getCurrentPosition();

        double deltaPos = newPos - firstPos;

        telemetry.addData("Status", "Encoder Change: " + deltaPos);
        telemetry.update();
    }
}
