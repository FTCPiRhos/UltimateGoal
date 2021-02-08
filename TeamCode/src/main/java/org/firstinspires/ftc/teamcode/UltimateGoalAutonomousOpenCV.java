package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Ultimate Goal OpenCV Blue Left", group="PiRhos")
//@Disabled
public class UltimateGoalAutonomousOpenCV extends UltimateGoalAutonomousBaseOpenCV {

    @Override
    public void runOpMode() {

        initHardware(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        OpenCVTestPipelineComp2.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        if(objectFound.equals(StarterStackDeterminationPipeline.RingPosition.FOUR)) {
            moveFwdAndBackForMilliseconds(0.4, 3000);
            moveSidewayForMilliseconds(0.4,600);
            sleep(1000);
            moveFwdAndBackForMilliseconds(-0.4, 1500);
            moveSidewayForMilliseconds(-0.4, 1200);
            sleep(1000);
            moveFwdAndBackForMilliseconds(0.4, 500);
//            moveFwdAndBack(0.4, 1440, 1440);
        }
        else if(objectFound.equals(StarterStackDeterminationPipeline.RingPosition.ONE)) {
            moveFwdAndBackForMilliseconds(0.4, 2500);
            moveSidewayForMilliseconds(-0.4,575);
            sleep(1000);
            moveFwdAndBackForMilliseconds(-0.4, 1000);
            sleep(1000);
            moveFwdAndBackForMilliseconds(0.4, 500);
        }
        else if(objectFound.equals(StarterStackDeterminationPipeline.RingPosition.NONE)) {
            moveFwdAndBackForMilliseconds(0.4, 2000);
            moveSidewayForMilliseconds(0.4,575);
            sleep(1000);
            moveFwdAndBackForMilliseconds(-0.4, 500);
            moveSidewayForMilliseconds(-0.4, 1500);
            sleep(1000);
            moveFwdAndBackForMilliseconds(0.4, 500);
        }
    }
}