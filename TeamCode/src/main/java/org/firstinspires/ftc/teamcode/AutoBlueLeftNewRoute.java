package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Blue Left New Route", group="PiRhos")
public class AutoBlueLeftNewRoute extends UltimateGoalAutonomousBaseOpenCV{

    @Override
    public void runOpMode() {

        // Initialize hardware
        initHardware(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double shooterPwr = -0.49 ;

        // Find number of rings + print for drivers
        OpenCVTestPipeline.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        // sleep(30000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();
        //set flywheel power at beginning to speed up set time
        flywheelShooter.setPower(shooterPwr);
        // move fwd and right to shoot 3 shots
        moveWPID(0,-54);
        moveWPID(-6,0);


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
        if ( objectFound.equals(OpenCVTestPipeline.RingPosition.NONE)){
            // flywheelShooter.setPower(shooterPwr);
            moveWPID(6, -6);
            sleep(500);

        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.ONE) ){
            // flywheelShooter.setPower(shooterPwr);
            moveWPID(-12, -84);
            sleep(500);
            moveWPID(18, 20);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)){
            // flywheelShooter.setPower(shooterPwr);
            moveWPID(12, -108);
            sleep(500);
            moveWPID(-6, 44);
        }

        // Shoot trigger


        // Park on white line
        moveWPID(-20, -12);

    }
}
