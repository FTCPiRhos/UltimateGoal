package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Red Left New Open CV", group="PiRhos")

public class AutoRedLeftNewOpenCV extends AutoBaseOpenCVShoot {
    class SetRPMVars {
        ElapsedTime timer = new ElapsedTime();
        boolean atTarget = false;
        boolean inWhile = false;
        boolean isPowershot = false;
        double pwrMul = 1.0;
        double curPower;
        double errorRPM;
        double curTime;
        double deltaError;

        double time_step = 25;

        double time_step_mul = time_step / 50.0;

        double kp = 0.0025 * 1;
        double ki = (0.0025 / 50.0) * 0.1 * 1;
        double kd = 0.0005 * 1;


        double lastErr = 0;
        double integralErr = 0;
        int inLockCount = 0;
        int loop_count = 0;
    }

    private AutoRedLeftNewOpenCV.SetRPMVars shooterRPMVars = new AutoRedLeftNewOpenCV.SetRPMVars();

    class MoveWPIDVars {
        ElapsedTime timer = new ElapsedTime();
        boolean inMove = false;
        double targetXCount;
        double targetYCount;

        double initialYPos;
        double initialXPos;

        double targetXPos;
        double targetYPos;

        double currentXPos;
        double currentYPos;
        double kp = 0.001;
        double ki = 0.00035;
        double kd = 0.02;
        double integralX = 0;
        double integralY = 0;
        double finalGain = 5;

        double errorX;
        double errorY;

        boolean movementDoneX = (Math.abs(errorX) < 25);
        boolean movementDoneY = (Math.abs(errorY) < 25);

        double lastXError;
        double lastYError;
        double curPowerX;
        double curPowerY;
        double capPowerX;
        double capPowerY;
        double minPowerX;
        double minPowerY;
        double deltaKX = 1;
        double deltaKY = 1;
        boolean firstPass = true;
        boolean fPosX = (errorX >= 0);
        boolean fposY = (errorY >= 0);

        double curPowerLF = 0;
        double curPowerLB = 0;
        double curPowerRF = 0;
        double curPowerRB = 0;
        double constantPowerX;
        double constantPowerY;

        double deltaXError;
        double deltaYError;
        double curTime;
        double derivativeX;
        double derivativeY;

        double deltaXPower;
        double deltaYPower;

        double powerLowThreshMul;
        double usePwrX;
        double usePwrY;

        double PwrRatioX;
        double PwrRatioY;

        double rampMul;
        double posBL;
        double posBR;
        double posFL;
        double posFR;
        double loopcount;
    }

    private AutoRedLeftNewOpenCV.MoveWPIDVars moveWPIDVars = new AutoRedLeftNewOpenCV.MoveWPIDVars();
    // target shooter speed
    double targetRPMGoal = -171;
    double flywheelPower = 0.6;
    double commandCount = 0;
    boolean part1 = true;
    double drivePwrStandard = 0.4;
    double rightBlockerRestPos = 0.62;
    double leftBlockerRestPos = 0.935;
    double rightBlockerBlockingPos = 1;
    double leftBlockerBlockingPos = 0.635;
    double prevCommandCount = 0;
    int loopCount = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        armServo.setPosition(1);
        ringBlockerLeft.setPosition(leftBlockerRestPos);
        ringBlockerRight.setPosition(rightBlockerRestPos);

        waitForStart();


        // Find number of rings + print for drivers
        OpenCVPipelineShoot.RingPosition objectFound = OpenCVRecognizeStack(1000);

        telemetry.addData("Object Found: ", objectFound);
        telemetry.update();

        while (opModeIsActive()) {
            loopCount++;
            flywheelPower = SetRPMnew(targetRPMGoal, flywheelPower);


            // Move fwd and left to shoot 3 shots
            if (commandCount == 0) {

                // forward and left to avoid ring stack
                moveWPIDnew(5, -75, 0.35);
                if (!moveWPIDVars.inMove){
                    commandCount++;
                    sleep(250);
                }

            }


            if (commandCount == 1) {
                if (prevCommandCount != commandCount){
                    telemetry.addData("command count = ", commandCount);
                    telemetry.addData("loop count = ", loopCount);
                    telemetry.update();
                    prevCommandCount = commandCount;


                }

                // strafe right to align to goal
                moveWPIDnew(-30, 0, 0.35);
                if (!moveWPIDVars.inMove) {
                    //rotate(2, .25);

                    commandCount++;
                }

            }


            if ((commandCount == 2) && (loopCount > 70)) {
                //if (shooterRPMVars.atTarget == true) {
                // shoot 3 rings
                if (prevCommandCount != commandCount) {
                    telemetry.addData("command count = ", commandCount);
                    telemetry.addData("loop count = ", loopCount);
                    telemetry.update();
                    prevCommandCount = commandCount;


                }
                shooterTrigger3xNPnew(flywheelPower);

                if (!moveWPIDVars.inMove) commandCount++;
                // }

            }


            // Drop off the wobble goal to specific box + align to goal


            if (objectFound.equals(OpenCVTestPipeline.RingPosition.NONE)) {
                if (commandCount == 3) {

                    // turn clockwise 90
                    rotateOdo(90, .3);


                    // move forward and left to drop off goal
                    moveWPID(20, -11, 0.40);


                    // arm down
                    ArmEncodersNew(0.75, 1350, 10000);
                    sleep(500);
                    // claw open
                    armServo.setPosition(0);


                    sleep(250);
                    // move right and back to avoid goal
                    moveWPID(13, 12, 0.3);
                    //moveWPID(0,2,0.5);



                    // strafe right to pick up wobble goal
                    moveWPID(-114, 0, 0.6);
                    //moveWPID(0,-5,0.4);



                    // close claw
                    armServo.setPosition(1);
                    sleep(1000);


                    // strafe left and forward to frop of goal # 2
                    moveWPID(103, -12, 0.6);



                    // open claw
                    armServo.setPosition(0);
                    sleep(500);

                    // strafe left to avoid goal
                    moveWPID(8, 0, 0.6);



                    // arm up
                    ArmEncodersNew(0.5, -1350, 10000);
                    // claw close
                    armServo.setPosition(1);


                    // park
                    moveWPID(-3, 15, 0.4);

                    sleep(500);
                    stop();
                }


            }
            else if (objectFound.equals(OpenCVTestPipeline.RingPosition.ONE)) {
                if (commandCount == 3) {
                    // move right and forward to drop goal
                    moveWPID(-14, 0, drivePwrStandard);
                    moveWPID(0, -24, drivePwrStandard);

                    // arm down

                    ArmEncodersNew(0.75, 1350, 10000);
                    sleep(750);
                    // open claw
                    armServo.setPosition(0);
                    // strafe left to avoid goal
                    moveWPID(18, 0, drivePwrStandard);
                    sleep(250);
                    // set shooter power for 4th shot
                    flywheelShooter.setPower(flywheelPower * 1.0125);
                    // intake on
                    intakeOnFast();

                    // move back to pick up ring
                    moveWPID(0, 44, drivePwrStandard);
                    sleep(2000);


                    sleep(1000);


                    // shoot
                    flywheelServo.setPosition(1);
                    sleep(500);
                    flywheelServo.setPosition(0.6);
                    sleep(500);
                    flywheelShooter.setPower(0);

                    // turn clockwise
                    rotateOdo(82, .4);


                    // move forward then strafe right to pick up goal # 2
                    //moveWPID(0,2,0.3);
                    moveWPID(-63, 0, drivePwrStandard);
                    // close claw
                    armServo.setPosition(1);
                    sleep(1000);
                    // arm up
                    ArmEncodersNew(1.0,-1350,1000);
                    sleep(1000);
                    // move back
                    moveWPID(0, 8, drivePwrStandard);
                    // strafe right to drop
                    moveWPID(146, 0, 0.7);
                    // arm down
                    ArmEncodersNew(1.0,1350,1000);
                    sleep(250);
                    // open claw
                    armServo.setPosition(0);
                    sleep(500);
                    // strafe left to void goal # 2
                    moveWPID(12, 0, drivePwrStandard);
                    // arm up
                    ArmEncodersNew(1.0, -1350, 10000);
                    // claw closed
                    armServo.setPosition(1);
                    // strafe to park
                    moveWPID(-38, 0, drivePwrStandard);

                    stop();
                }
            }
            else if (objectFound.equals(OpenCVTestPipeline.RingPosition.FOUR)) {
                if (commandCount == 3) {
                    // intake on reverse power
                    intakeReverse();
                    // set shooter power
                    flywheelShooter.setPower(flywheelPower * 1.035);
                    // strafe left
                    moveWPID(10, 0, 0.4);

                    // move back to knock down stack
                    moveWPID(0, 15, 0.35);

                    // move forward
                    moveWPID(0, -5, 0.4);

                    // intake on to pickup
                    intakeOnFast();
                    // move back to pick up rings
                    moveWPID(0, 16, 0.15);
                    sleep(1500);


                    // move forward
                    moveWPID(0, -6, 0.4);
                    // strafe right to shoot
                    moveWPID(-6, 0, 0.4);





                    // shoot 3 picked up rings
                    for (int i = 0; i < 3; i += 1) {

                        flywheelServo.setPosition(1);
                        sleep(350);
                        intakeOnFast();

                        if (i == 0) flywheelShooter.setPower(flywheelPower * 1.08 * 1.04);

                        if (i == 1) flywheelShooter.setPower(flywheelPower * 1.08 * 1.035);
                        flywheelServo.setPosition(0.6);
                        sleep(350);


                    }
                    // setting power to shooter
                    flywheelShooter.setPower(flywheelPower * 1.05);
                    // intake on (irrelevant i think but dont change)
                    intakeOnFast();
                    // move back to pick up rings
                    moveWPID(0, 12, 0.35);
                    sleep(1000);
                    // move forward to shoot
                    moveWPID(0, -20, 0.4);

                    sleep(250);
                    // shoot 2 rings
                    for (int i = 0; i < 2; i++) {
                        flywheelServo.setPosition(1);
                        sleep(350);
                        intakeOnFast();
                        flywheelServo.setPosition(0.6);
                        sleep(350);
                        //intakeOff();
                        flywheelShooter.setPower(flywheelPower * 1.05);


                    }
                    // turn off shooter
                    flywheelShooter.setPower(0);

                    // rotate clockwise
                    rotateOdo(82.5, 0.4);
                    // forwards and strafe right to drop off wobble goal
                    moveWPID(115, -12, 0.6);
                    // arm down
                    ArmEncodersNew(1, 1350, 10000);
                    sleep(250);
                    // claw open
                    armServo.setPosition(0);
                    // strafe left to avoid goal
                    moveWPID(8, 0,0.4);
                    // arm up
                    ArmEncodersNew(1, -1350, 10000);
                    // move back and strafe right to park
                    moveWPID(0, 14, 0.4);
                    moveWPID(-75,0,0.5);
                    stop();


                    // the rest isnt used

                    // open arm servo
                    //rotate(-2, 1.0);
                    // end of park
                    moveWPID(0,-12,1);
                    moveWPID(-160,0,0.4);

                    // close servo
                    armServo.setPosition(1);
                    sleep(1000);
                    moveWPID(172,0,0.4);
                    armServo.setPosition(0);
                    sleep(350);
                    moveWPID(14, 0, drivePwrStandard);
                    ArmEncodersNew(1.0,-1350,1000);
                    moveWPID(-60, 8, 0.4);







                    commandCount++;
                    stop();
                }

            }




        }
    }

    public void moveWPIDnew(double targetXInches, double targetYInches, double maxPwr) {

        if (!moveWPIDVars.inMove) {
            frontLeft.setPower(0);
            frontRight.setPower(0);

            backRight.setPower(0);
            backLeft.setPower(0);


            moveWPIDVars.targetXCount = targetXInches * COUNTS_PER_INCH;
            moveWPIDVars.targetYCount = targetYInches * COUNTS_PER_INCH;

            moveWPIDVars.initialYPos = (backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
            moveWPIDVars.initialXPos = (backRight.getCurrentPosition() - backLeft.getCurrentPosition()) / 2;

            moveWPIDVars.targetXPos = moveWPIDVars.targetXCount + moveWPIDVars.initialXPos;
            moveWPIDVars.targetYPos = moveWPIDVars.targetYCount + moveWPIDVars.initialYPos;

            moveWPIDVars.currentXPos = moveWPIDVars.initialXPos;
            moveWPIDVars.currentYPos = moveWPIDVars.initialYPos;
            double kp = moveWPIDVars.kp;
            double ki = moveWPIDVars.ki;
            double kd = moveWPIDVars.kd;


            moveWPIDVars.errorX = moveWPIDVars.targetXPos - moveWPIDVars.currentXPos;
            moveWPIDVars.errorY = moveWPIDVars.targetYPos - moveWPIDVars.currentYPos;

            moveWPIDVars.movementDoneX = (Math.abs(moveWPIDVars.errorX) < 25);
            moveWPIDVars.movementDoneY = (Math.abs(moveWPIDVars.errorY) < 25);

            moveWPIDVars.lastXError = 0;
            moveWPIDVars.lastYError = 0;
            moveWPIDVars.curPowerX = 0;
            moveWPIDVars.curPowerY = 0;
            moveWPIDVars.capPowerX = maxPwr;
            moveWPIDVars.capPowerY = maxPwr;
            moveWPIDVars.minPowerX = 0;
            moveWPIDVars.minPowerY = 0;
            moveWPIDVars.deltaKX = 1;
            moveWPIDVars.deltaKY = 1;
            moveWPIDVars.firstPass = true;
            moveWPIDVars.fPosX = (moveWPIDVars.errorX >= 0);
            moveWPIDVars.fposY = (moveWPIDVars.errorY >= 0);

            moveWPIDVars.curPowerLF = 0;
            moveWPIDVars.curPowerLB = 0;
            moveWPIDVars.curPowerRF = 0;
            moveWPIDVars.curPowerRB = 0;
            moveWPIDVars.inMove = true;


        } else if (moveWPIDVars.inMove) {

            moveWPIDVars.constantPowerX = moveWPIDVars.fPosX ? 0.20 : -0.20;
            moveWPIDVars.constantPowerY = moveWPIDVars.fposY ? 0.20 : -0.20;

            if ((Math.abs(targetXInches) + Math.abs(targetYInches)) < 25) {
                moveWPIDVars.capPowerX = maxPwr;
                moveWPIDVars.capPowerY = maxPwr;
            }
            moveWPIDVars.timer = new ElapsedTime();
            // start loop while any error is > some number
            if ((!moveWPIDVars.movementDoneX || !moveWPIDVars.movementDoneY)) {


                moveWPIDVars.deltaXError = moveWPIDVars.firstPass ? 0 : moveWPIDVars.errorX - moveWPIDVars.lastXError;
                moveWPIDVars.deltaYError = moveWPIDVars.firstPass ? 0 : moveWPIDVars.errorY - moveWPIDVars.lastYError;

                moveWPIDVars.firstPass = false;

                moveWPIDVars.curTime = moveWPIDVars.timer.time();

                moveWPIDVars.integralX += moveWPIDVars.errorX * moveWPIDVars.curTime;
                moveWPIDVars.integralY += moveWPIDVars.errorY * moveWPIDVars.curTime;

                moveWPIDVars.derivativeX = moveWPIDVars.deltaXError / moveWPIDVars.curTime;
                moveWPIDVars.derivativeY = moveWPIDVars.deltaYError / moveWPIDVars.curTime;


                moveWPIDVars.timer.reset();

                if (moveWPIDVars.movementDoneX) moveWPIDVars.deltaKX = 0;
                if (moveWPIDVars.movementDoneY) moveWPIDVars.deltaKY = 0;

                moveWPIDVars.deltaXPower = moveWPIDVars.deltaKX * ((moveWPIDVars.errorX * moveWPIDVars.kp) + (moveWPIDVars.integralX * moveWPIDVars.ki) + (moveWPIDVars.derivativeX * moveWPIDVars.kd));
                moveWPIDVars.deltaYPower = moveWPIDVars.deltaKY * ((moveWPIDVars.errorY * moveWPIDVars.kp) + (moveWPIDVars.integralY * moveWPIDVars.ki) + (moveWPIDVars.derivativeY * moveWPIDVars.kd));


                moveWPIDVars.curPowerX = moveWPIDVars.finalGain * (moveWPIDVars.deltaXPower * 0.8 + moveWPIDVars.constantPowerX);
                moveWPIDVars.curPowerY = moveWPIDVars.finalGain * (moveWPIDVars.deltaYPower * 0.8 + moveWPIDVars.constantPowerY);
                moveWPIDVars.powerLowThreshMul = 0;

                if (((Math.abs(moveWPIDVars.curPowerX)) > moveWPIDVars.minPowerX) || ((Math.abs(moveWPIDVars.curPowerY)) > moveWPIDVars.minPowerY))
                    moveWPIDVars.powerLowThreshMul = 1;

                moveWPIDVars.usePwrX = moveWPIDVars.powerLowThreshMul * moveWPIDVars.curPowerX;
                moveWPIDVars.usePwrY = moveWPIDVars.powerLowThreshMul * moveWPIDVars.curPowerY;

                if (moveWPIDVars.curPowerX > moveWPIDVars.capPowerX)
                    moveWPIDVars.usePwrX = moveWPIDVars.capPowerX;
                if (moveWPIDVars.curPowerX < (-1 * moveWPIDVars.capPowerX))
                    moveWPIDVars.usePwrX = -1 * moveWPIDVars.capPowerX;

                if (moveWPIDVars.curPowerY > moveWPIDVars.capPowerY)
                    moveWPIDVars.usePwrY = moveWPIDVars.capPowerY;
                if (moveWPIDVars.curPowerY < (-1 * moveWPIDVars.capPowerY))
                    moveWPIDVars.usePwrY = -1 * moveWPIDVars.capPowerY;

                moveWPIDVars.PwrRatioX = (moveWPIDVars.curPowerX != 0) ? Math.abs(moveWPIDVars.usePwrX / moveWPIDVars.curPowerX) : 0;
                moveWPIDVars.PwrRatioY = (moveWPIDVars.curPowerY != 0) ? Math.abs(moveWPIDVars.usePwrY / moveWPIDVars.curPowerY) : 0;

                if (moveWPIDVars.PwrRatioX != moveWPIDVars.PwrRatioY) {
                    if ((moveWPIDVars.PwrRatioX != 0) && (moveWPIDVars.PwrRatioX < moveWPIDVars.PwrRatioY)) {
                        moveWPIDVars.usePwrY = moveWPIDVars.PwrRatioX * moveWPIDVars.usePwrY / moveWPIDVars.PwrRatioY;
                    }
                    if ((moveWPIDVars.PwrRatioY != 0) && (moveWPIDVars.PwrRatioY < moveWPIDVars.PwrRatioX)) {
                        moveWPIDVars.usePwrX = moveWPIDVars.PwrRatioY * moveWPIDVars.usePwrX / moveWPIDVars.PwrRatioX;
                    }

                }

                moveWPIDVars.usePwrX = (!moveWPIDVars.movementDoneX) ? moveWPIDVars.usePwrX : 0;
                moveWPIDVars.usePwrY = (!moveWPIDVars.movementDoneY) ? moveWPIDVars.usePwrY : 0;

                moveWPIDVars.curPowerLF = moveWPIDVars.usePwrY + moveWPIDVars.usePwrX;
                moveWPIDVars.curPowerLB = moveWPIDVars.usePwrY - moveWPIDVars.usePwrX;
                moveWPIDVars.curPowerRF = moveWPIDVars.usePwrY - moveWPIDVars.usePwrX;
                moveWPIDVars.curPowerRB = moveWPIDVars.usePwrY + moveWPIDVars.usePwrX;

                backRight.setPower(moveWPIDVars.curPowerRB);
                backLeft.setPower(moveWPIDVars.curPowerLB);

                frontLeft.setPower(moveWPIDVars.curPowerLF);
                frontRight.setPower(moveWPIDVars.curPowerRF);

                sleep(50);

                moveWPIDVars.posBL = backLeft.getCurrentPosition();
                moveWPIDVars.posBR = backRight.getCurrentPosition();

                moveWPIDVars.posFL = frontLeft.getCurrentPosition();
                moveWPIDVars.posFR = frontRight.getCurrentPosition();

                moveWPIDVars.currentYPos = (moveWPIDVars.posBL + moveWPIDVars.posBR) / 2;
                moveWPIDVars.currentXPos = (moveWPIDVars.posBR - moveWPIDVars.posBL) / 2;

                moveWPIDVars.errorX = (moveWPIDVars.targetXPos - moveWPIDVars.currentXPos);
                moveWPIDVars.errorY = (moveWPIDVars.targetYPos - moveWPIDVars.currentYPos);


                moveWPIDVars.lastXError = moveWPIDVars.errorX;
                moveWPIDVars.lastYError = moveWPIDVars.errorY;

                moveWPIDVars.movementDoneX = (Math.abs(moveWPIDVars.errorX) < 100) || moveWPIDVars.movementDoneX || (moveWPIDVars.fPosX && moveWPIDVars.errorX < 0) || (!moveWPIDVars.fPosX && moveWPIDVars.errorX > 0);
                moveWPIDVars.movementDoneY = (Math.abs(moveWPIDVars.errorY) < 100) || moveWPIDVars.movementDoneY || (moveWPIDVars.fposY && moveWPIDVars.errorY < 0) || (!moveWPIDVars.fposY && moveWPIDVars.errorY > 0);


                telemetry.addData("ErrX = ", moveWPIDVars.errorX);
                telemetry.addData("ErrY = ", moveWPIDVars.errorY);
                telemetry.addData("Front Left Encoder =", moveWPIDVars.posFL);
                telemetry.addData("Front Right Encoder ", moveWPIDVars.posFR);
                telemetry.addData("Back Left Encoder", moveWPIDVars.posBL);
                telemetry.addData("Back Right Encoder =", moveWPIDVars.posBR);
                telemetry.addData("Power ratio x =", moveWPIDVars.PwrRatioX);
                telemetry.addData("Power ratio y =", moveWPIDVars.PwrRatioY);


                telemetry.update();

                return;
            }

            moveWPIDVars.rampMul = 1.0;

            for (int i = 0; i < 5; i++) {
                moveWPIDVars.rampMul -= 0.2;
                frontLeft.setPower(moveWPIDVars.curPowerLF * moveWPIDVars.rampMul);
                frontRight.setPower(moveWPIDVars.curPowerRF * moveWPIDVars.rampMul);

                backRight.setPower(moveWPIDVars.curPowerRB * moveWPIDVars.rampMul);
                backLeft.setPower(moveWPIDVars.curPowerLB * moveWPIDVars.rampMul);
                sleep(10);
            }
            moveWPIDVars.inMove = false;
        }




    }


    public double SetRPMnew ( double targetRPM, double motorPower){
        //  if (!shooterRPMVars.isValid) return motorPower;

        if (!shooterRPMVars.inWhile) {
            double pwrMul = shooterRPMVars.pwrMul;

            double time_step = shooterRPMVars.time_step;

            double time_step_mul = shooterRPMVars.time_step_mul;

            double kp = shooterRPMVars.kp;
            double ki = shooterRPMVars.ki;
            double kd = shooterRPMVars.kd;


            shooterRPMVars.errorRPM = targetRPM + getRPM(time_step);
            shooterRPMVars.curPower = motorPower;
            shooterRPMVars.lastErr = 0;
            shooterRPMVars.integralErr = 0;
            shooterRPMVars.inLockCount = 0;
            shooterRPMVars.loop_count = 0;
            shooterRPMVars.inWhile = true;
            shooterRPMVars.curTime = shooterRPMVars.timer.time();
            shooterRPMVars.timer.reset();
        } else {

            //              while (loop_count < 1000) {
            shooterRPMVars.deltaError = shooterRPMVars.errorRPM - shooterRPMVars.lastErr;
            shooterRPMVars.lastErr = shooterRPMVars.errorRPM;
            double time_int = shooterRPMVars.timer.time();
            shooterRPMVars.timer.reset();

            double derivative = shooterRPMVars.deltaError / time_int;


            if (Math.abs(shooterRPMVars.errorRPM) < 5) {
                shooterRPMVars.integralErr += shooterRPMVars.errorRPM * time_int;
            } else {
                shooterRPMVars.integralErr += 0;
//                integralErr += ((errorRPM > 0) ? 5 * time_int : -5 * time_int) ;
            }

            double deltaPower = -1 * shooterRPMVars.time_step_mul * ((shooterRPMVars.errorRPM * shooterRPMVars.kp) + (shooterRPMVars.integralErr * shooterRPMVars.ki) + (derivative * shooterRPMVars.kd));

            /* double pwrMul = (Math.abs(errorRPM) > 20) ? 1.0 :
                            (Math.abs(errorRPM) > 10)  ? 1.0/4.0 :
                            (Math.abs(errorRPM) > 5)  ? 1.0/16.0 :
                                    (Math.abs(errorRPM) > 2.5)  ? 01.0/64.0 : (1.0/128.0) ;

             */
            shooterRPMVars.curPower += (deltaPower * shooterRPMVars.pwrMul);

            if (shooterRPMVars.curPower > 0.7) shooterRPMVars.curPower = 0.7;
            if (shooterRPMVars.curPower < -0.7) shooterRPMVars.curPower = -0.7;

            flywheelShooter.setPower(shooterRPMVars.curPower);
            double RPM = getRPM(shooterRPMVars.time_step);
            shooterRPMVars.errorRPM = targetRPM + RPM;
            /*
            telemetry.addData("RPM = ", RPM);
            telemetry.addData("errorRPM = ", errorRPM);
            telemetry.addData("curPower  = ", curPower);
            telemetry.addData("deltaPower  = ", deltaPower);
            telemetry.update();

             */

            if (Math.abs(shooterRPMVars.errorRPM) <2.5) {
                if (shooterRPMVars.inLockCount > 1) {
                    shooterRPMVars.pwrMul = 1.0;
                }
                shooterRPMVars.inLockCount += 1;
                if (shooterRPMVars.inLockCount > 5) {
                    // shooterRPMVars.inWhile = false;
                    shooterRPMVars.atTarget = true;
                    return (shooterRPMVars.curPower);

                }
            } else {
                shooterRPMVars.inLockCount = 0;
                shooterRPMVars.pwrMul = 1.0;
                shooterRPMVars.atTarget = false;
            }
            //               }
        }
        if (shooterRPMVars.loop_count > 1000) {
            //shooterRPMVars.inWhile = false;
            //shooterRPMVars.isValid = false;

        }
        return (shooterRPMVars.curPower);
    }

}


