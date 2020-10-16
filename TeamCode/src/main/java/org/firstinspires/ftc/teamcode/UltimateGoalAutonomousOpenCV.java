package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Ultimate Goal OpenCV", group="PiRhos")
//@Disabled
public class UltimateGoalAutonomousOpenCV extends UltimateGoalAutonomousBase {

    @Override
    public void runOpMode() {

        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        StarterStackDeterminationPipeline.RingPosition objectFound = OpenCVRecognizeStack(3000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        if(objectFound.equals(StarterStackDeterminationPipeline.RingPosition.FOUR))
            moveFwdAndBackForMilliseconds(0.4, 1000);
        else if(objectFound.equals(StarterStackDeterminationPipeline.RingPosition.ONE))
            moveFwdAndBackForMilliseconds(-0.4, 1000);
        else if(objectFound.equals(StarterStackDeterminationPipeline.RingPosition.NONE))
            moveSidewayForMilliseconds(0.4, 1000);


    }
}