package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Simple Testing", group="PiRhos")
//@Disabled
public class SimpleTestingForEncoders extends UltimateGoalAutonomousBaseOpenCV {

    @Override
    public void runOpMode() {


        initHardware(true);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //String objectFound = TFRecognizeStack(3000);
        //telemetry.addData("Object Found: ", objectFound);
        CommonMethodForArm();




        stop();
    }
}