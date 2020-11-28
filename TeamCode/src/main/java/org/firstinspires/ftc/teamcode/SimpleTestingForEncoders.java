package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Simple Testing", group="PiRhos")
//@Disabled
public class SimpleTestingForEncoders extends RotationalMethods {

    @Override
    public void runOpMode() {


        initHardware();




        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //String objectFound = TFRecognizeStack(3000);
        //telemetry.addData("Object Found: ", objectFound);
        rotate(60, 0.3);



        stop();
    }
}