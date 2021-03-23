package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Red Left", group="PiRhos")
public class Auto_Red_Left extends UltimateGoalAutonomousBaseOpenCV{

    @Override
    public void runOpMode() {

        // Initialize hardware
        initHardware(true);

       // flywheelServo.setPosition(1);
        armServo.setPosition(1);
       // flywheelServo.setPosition(0.55);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
       // double targetRPM = -125 ;
        //double flywheelPower = 0;

        // Find number of rings + print for drivers
        OpenCVTestPipelineComp2.RingPosition objectFound = OpenCVRecognizeStack(1000 );
        sleep(30000);
        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        // Set flywheel power at beginning to speed up set time
        flywheelShooter.setPower(0);

        // Move fwd and left to shoot 3 shots
        moveWPID(5,-57,0.75);
        moveWPID(-22,0,0.75);
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
        double flywheelPwr = shooterTrigger3xNP();



        // Drop off the wobble goal to specific box + align to goal
        if ( objectFound.equals(OpenCVTestPipeline.RingPosition.NONE) ){
            rotate(90, .8);

            moveWPID(5, -3,0.75);
            ArmEncodersNew(0.5, 1350, 10000);
            sleep(500);
            armServo.setPosition(0);

            moveWPID(8, 0,0.75);
           // CommonMethodForArm();
            //sleep(100);
            moveWPID(-3, 5,0.75);
            moveWPID(-70,0,0.75);
            armServo.setPosition(1);
            sleep(1000);
            moveWPID(64,-5,0.75);
            armServo.setPosition(0);
            sleep(500);
            moveWPID(8, 0,0.75);
            ArmEncodersNew(0.3, -1350, 10000);
            armServo.setPosition(1);
            moveWPID(-3, 5,0.75);



        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.ONE) ){

            moveWPID(-6,0,0.75);
            moveWPID(0,-7,0.75);

            //  CommonMethodForArm();
            ArmEncodersNew(0.75, 1350, 10000);
            sleep(750);
            armServo.setPosition(0);
            moveWPID(16, 0,0.75);
            sleep(250);
            flywheelShooter.setPower(flywheelPwr * 1.06);

            intakeOn();
           // flywheelShooter.setPower(flywheelPwr * 1.05);

            //moveWPID(8,0,0.75);
            moveWPID(0,32,0.75);
            sleep(1000);

            intakeOff();

            sleep(1000);

            rotate(3, .5);


            flywheelServo.setPosition(1);
            sleep(500);
            flywheelServo.setPosition(0.6);
            sleep(500);
            flywheelShooter.setPower(0);

            // moveWPID(10,0,0.3);
            rotate(85, .8);


            // 4 back
            // 36 side

           // moveWPID(0,4,0.3);
            moveWPID(-30,0,0.75);
            armServo.setPosition(1);
            sleep(1000);
            moveWPID(0,13,0.75);
            moveWPID(100,0,1.0);
            armServo.setPosition(0);
            sleep(500);
            moveWPID(12, 0,0.75);
            ArmEncodersNew(1.0, -1350, 10000);
            armServo.setPosition(1);

            moveWPID(-28,0,1.0);



           // moveWPID(-8, -12,0.75);
            //sleep(100);
          //  moveWPID(0, 1,0.75);
        }
        else if ( objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)){
            intakeReverse();

            flywheelShooter.setPower(flywheelPwr * 1.085);
            moveWPID(7,0,0.5);


            moveWPID(0,18,1.0);

            intakeOff();

            moveWPID(0,-5,0.75);

            /*

            backLeft.setPower(0.8);
            frontLeft.setPower(0.8);
            backRight.setPower(0.8);
            frontRight.setPower(0.8);
            sleep(300);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);

            sleep(250);

             */




            intakeOnFast();
            moveWPID(0,16,0.2);
            sleep(1500);
            /*
            backLeft.setPower(0.3);
            frontLeft.setPower(0.3);
            backRight.setPower(0.3);
            frontRight.setPower(0.3);
            sleep(1500);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);

             */


            intakeReverse();

            //moveWPID(0,-26,0.75);
            moveWPID(0,-4,0.75);
            moveWPID(-8,0,0.75);

            intakeOff();

            for (int i = 0 ; i < 3 ; i += 1) {

                flywheelServo.setPosition(1);
                sleep(350);
                intakeOn();

                if (i == 0) flywheelShooter.setPower(flywheelPwr * 1.08 * 1.15);

                if (i == 1) flywheelShooter.setPower(flywheelPwr * 1.08 * 1.15);
                flywheelServo.setPosition(0.6);
                sleep(350);


            }
            flywheelShooter.setPower(flywheelPwr * 1.025);
            intakeOnFast();
            moveWPID(8,0,0.75);

            moveWPID(0,8,0.75);
            sleep(1000);
            intakeOff();
            moveWPID(0,-34,0.75);
            moveWPID(-16,0,0.75);

            sleep(250);

            for (int i = 0; i < 2 ; i++) {
                flywheelServo.setPosition(1);
                sleep(350);
                intakeOn();
                flywheelServo.setPosition(0.6);
                sleep(350);
                intakeOff();
                flywheelShooter.setPower(flywheelPwr * 1.05);


            }
            flywheelShooter.setPower(0);
            rotate(90, 1.0);
            moveWPID(56, 4,1.0);
            CommonMethodForArm();
            //sleep(100);
            moveWPID(-43, 8,1.0);
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

