package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue Left", group="PiRhos")
public class Auto_Blue_Left extends UltimateGoalAutonomousBaseOpenCV{

    @Override
    public void runOpMode() {

        // Initialize hardware
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Find number of rings + print for drivers
        StarterStackDeterminationPipeline.RingPosition objectFound = OpenCVRecognizeStack(1000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        // Drop off the wobble goal to specific box + align to goal
        if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.NONE) || true){
            moveWPID(0, -62);
            moveWPID(-12, 0);
            sleep(500);
            moveWPID(0, 12);
            moveWPID(12, 0);
        }
        else if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.ONE) ){
            moveWPID(-12, 99);
            sleep(500);
            moveWPID(12, -36);
        }
        else if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.FOUR) ){
            moveWPID(12, 123);
            sleep(500);
            moveWPID(-12, -60);
        }

        // Shoot
        SetRPM(133, -0.1);
        //sleep(500);

        // Park on white line
        //moveFwdAndBackForDistance(0.6, 12, 30000);

    }
}
