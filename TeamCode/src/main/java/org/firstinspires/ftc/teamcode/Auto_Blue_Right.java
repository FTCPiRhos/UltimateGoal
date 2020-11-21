package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue Right", group="PiRhos")
public class Auto_Blue_Right extends UltimateGoalAutonomousBaseOpenCV{

    @Override
    public void runOpMode() {

        // Initialize hardware
        initHardware(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double shooterPwr = -0.5157 ;

        // Find number of rings + print for drivers
        StarterStackDeterminationPipeline.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        sleep(10000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();


        // Drop off the wobble goal to specific box + align to goal
        /*if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.NONE) ){
            flywheelShooter.setPower(shooterPwr);
            moveWPID(12, -60, 1.05);
            sleep(500);
            //moveWPID(0, 0);
            // Shoot
        }
        else if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.ONE) ){
            flywheelShooter.setPower(shooterPwr);
            moveWPID(-24, -84, 1.025);
            sleep(500);
            moveWPID(24, 20, 1.05);
        }
        else if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.FOUR)){
            flywheelShooter.setPower(shooterPwr);
            moveWPID(12, -108, 1.01);
            sleep(500);

            moveWPID(-6, 44, 1.025);
        }

        shooterPwr = SetRPM(133.0, shooterPwr);

        telemetry.addData("Trigger", 1)  ;
        telemetry.update() ;
        sleep(2500);

        // Shoot trigger
        shooterPwr = SetRPM(133, shooterPwr);
        telemetry.addData("Trigger", 2)  ;
        telemetry.update() ;
        sleep(2500);


        shooterPwr = SetRPM(133, shooterPwr);
        telemetry.addData("Trigger", 3)  ;
        telemetry.update() ;
        sleep(2500);

        // Park on white line
        moveWPID(-20, -15, 105);
        //moveWPID(-12, 0);
*/
    }
}
