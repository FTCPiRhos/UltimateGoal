package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto Blue Right", group="PiRhos")
@Disabled public class Auto_Blue_Right extends UltimateGoalAutonomousBaseOpenCV{

    @Override
    public void runOpMode() {

        // Initialize hardware
        initHardware(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double shooterPwr = -0.49 ;

        // Find number of rings + print for drivers
        OpenCVTestPipelineComp2.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        // sleep(30000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        // Set flywheel power at beginning to speed up set time
        flywheelShooter.setPower(shooterPwr);

        // Move fwd and left to shoot 3 shots
        moveWPID(20,-54);

        shooterPwr = SetRPM(128, shooterPwr);
        telemetry.addData("Trigger", 1);
        telemetry.update();
        sleep(2500);

        shooterPwr = SetRPM(128, shooterPwr);
        telemetry.addData("Trigger", 2)  ;
        telemetry.update() ;
        sleep(2500);

        shooterPwr = SetRPM(128, shooterPwr);
        telemetry.addData("Trigger", 3)  ;
        telemetry.update() ;
        sleep(2500);

        // Drop off the wobble goal to specific box + align to goal
        if ( objectFound.equals(OpenCVTestPipeline.RingPosition.NONE) ){
            moveWPID(4, -6);
            sleep(500);
            moveWPID(-20, -12);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.ONE) ){
            moveWPID(-20, -30);
            sleep(500);
            moveWPID(4, 12);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)){
            moveWPID(4, -54);
            sleep(500);
            moveWPID(-20, 36);
        }

        // Move back to grab second wobble goal
        // moveWPID(20, 72);
        /*if ( objectFound.equals(OpenCVTestPipeline.RingPosition.NONE) ){
            moveWPID(0, -6);
            sleep(500);
            moveWPID(-20, -12);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.ONE) ){
            moveWPID(-24, -30);
            sleep(500);
            moveWPID(4, 12);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)){
            moveWPID(0, -54);
            sleep(500);
            moveWPID(-20, 36);
        }*/

    }
}
