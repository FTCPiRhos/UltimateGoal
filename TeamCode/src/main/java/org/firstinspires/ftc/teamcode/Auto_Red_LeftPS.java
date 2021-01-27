package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Red Left PS", group="PiRhos")
public class Auto_Red_LeftPS extends UltimateGoalAutonomousBaseOpenCV{

    @Override
    public void runOpMode() {

        // Initialize hardware
        initHardware(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double targetRPM = -125 ;
        double flywheelPower = 0;

        // Find number of rings + print for drivers
        OpenCVTestPipeline.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        //sleep(30000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        // Set flywheel power at beginning to speed up set time
        flywheelShooter.setPower(flywheelPower);

        // Move fwd and left to shoot 3 shots
        moveWPID(24,-60.5);
        Powershots();
        stop();

        /*
        flywheelPower = SetRPM(targetRPM, flywheelPower);
        flywheelPower = 1.0 * flywheelPower;
        shooterTrigger();
        //sleep(1500);

        flywheelPower = SetRPM(targetRPM, flywheelPower);
        flywheelPower = 1.0 * flywheelPower;
        shooterTrigger();
        //sleep(1500);

        flywheelPower = SetRPM(targetRPM, flywheelPower);
        flywheelPower = 1.0 * flywheelPower;
        shooterTrigger();
       // sleep(1500);

 */
        shooterTrigger3x();

        // Drop off the wobble goal to specific box + align to goal
        if ( objectFound.equals(OpenCVTestPipeline.RingPosition.NONE) ){
            rotate(90, .8);
            moveWPID(0, -30);
            CommonMethodForArm();
            //sleep(100);
            moveWPID(0, 30);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.ONE) ){
            moveWPID(-12, -24);
            CommonMethodForArm();
            //sleep(100);
            moveWPID(6, 18);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)){
            rotate(90, .8);
            moveWPID(46, -18);
            CommonMethodForArm();
            //sleep(100);
            moveWPID(-38, 18);
        }

        // Move back to grab second wobble goal
        // moveWPID(-20, 72);
        /*if ( objectFound.equals(OpenCVTestPipeline.RingPosition.NONE) ){
            moveWPID(0, -6);
            sleep(500);
            moveWPID(20, -12);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.ONE) ){
            moveWPID(24, -30);
            sleep(500);
            moveWPID(-4, 12);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)){
            moveWPID(0, -54);
            sleep(500);
            moveWPID(20, 36);
        }*/
    }
}

