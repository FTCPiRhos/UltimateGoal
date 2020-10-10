package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Ultimate Goal Move", group="PiRhos")
//@Disabled
public class UltimateGoalAutonomousMove extends UltimateGoalAutonomousBase {

    @Override
    public void runOpMode() {

        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        String objectFound = TFRecognizeStack(3000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        if (objectFound.equalsIgnoreCase(LABEL_NONE_ELEMENT))
            moveFwdAndBackForMilliseconds(0.4, 1000);
        else if(objectFound.equalsIgnoreCase(LABEL_FIRST_ELEMENT))
            moveFwdAndBackForMilliseconds(-0.4, 1000);
        else if(objectFound.equalsIgnoreCase(LABEL_SECOND_ELEMENT))
            moveSidewayForMilliseconds(0.4, 1000);

        /*telemetry.addData("Status: ", 1);
        telemetry.update();
        moveFwdAndBackForMilliseconds(0.4, 1000);
        sleep(500);*/

       /* telemetry.addData("Status: ", 2);
        telemetry.update();
        moveSidewayForMilliseconds( -0.6, 1000 );
        sleep(500);*/




        // Vuforia and Tensorflow related clean-up
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}