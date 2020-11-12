package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Simple Testing", group="PiRhos")
//@Disabled
public class SimpleTestingForEncoders extends UltimateGoalAutonomousBase {

    @Override
    public void runOpMode() {

        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //String objectFound = TFRecognizeStack(3000);
        //telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

       moveForwardandBackwardEncoders(0.7, 35, 10000);
       //moveSideWaysEncoders(0.6, 40,10000);
       //moveSideWaysEncoders(-0.6, -40,10000);
        //moveForwardandBackwardEncoders(-0.7, -35, 10000);

    }
}