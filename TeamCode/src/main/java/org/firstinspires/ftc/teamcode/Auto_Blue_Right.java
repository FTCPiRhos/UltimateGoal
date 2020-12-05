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
        double shooterPwr = -0.5157;

        // Find number of rings + print for drivers
        OpenCVTestPipeline.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        // sleep(30000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        // Drop off the wobble goal to specific box + align to goal
        if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.NONE) ){
            flywheelShooter.setPower(shooterPwr);
            moveWPID(36, -60);
            sleep(500);
            moveWPID(-6, 0);
        }
        else if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.ONE) ){
            flywheelShooter.setPower(shooterPwr);
            moveWPID(12, -84);
            sleep(500);
            moveWPID(18, 20);
        }
        else if ( objectFound.equals(StarterStackDeterminationPipeline.RingPosition.FOUR)){
            flywheelShooter.setPower(shooterPwr);
            moveWPID(36, -108);
            sleep(500);
            moveWPID(-6, 44);
        }

        // Shoot trigger
        shooterPwr = SetRPM(133, shooterPwr);
        telemetry.addData("Trigger", 1)  ;
        telemetry.update() ;
        sleep(2500);

        shooterPwr = SetRPM(133, shooterPwr);
        telemetry.addData("Trigger", 2)  ;
        telemetry.update() ;
        sleep(2500);

        shooterPwr = SetRPM(133, shooterPwr);
        telemetry.addData("Trigger", 3)  ;
        telemetry.update() ;
        sleep(2500);

        // Park on white line
        moveWPID(-12, -12);
    }
}
