package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.USBAccessibleLynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="Flywheel New Test", group="PiRhos")


public class NewShooterTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Servo flywheelServo = null;
    private DcMotor flywheelShooter = null;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 / 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;   // For figuring circumference - 100mm
    static final double COUNTS_PER_INCH = 1.45 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_SLOW = 0.4;
    static final double DRIVE_SPEED = 0.7;

    @Override

    public void runOpMode() {
        initHardware();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */

        waitForStart();
        runtime.reset();
        telemetry.addLine("Started");
        telemetry.update();
        // powershot rpm = 125~
        // high tower rpm = 132~
        double targetRPM = -127.5 ;
        double flywheelPower = 0.47;
        boolean firstPass = true;
        while (opModeIsActive()) {
            if (firstPass) {
                flywheelShooter.setPower(flywheelPower);
                sleep(1000);
                firstPass = false;
            }
/*
            if (gamepad1.left_bumper) {
                flywheelServo.setPosition(0.5);
                sleep(500);
                flywheelServo.setPosition(0.9375);
            }

 */



            //for (int i =0 ; i < 5 ; i = i + 1) {

            for (int i = 0 ; i < 3 ; i += 1) {
                flywheelPower = SetRPM(targetRPM, flywheelPower);
                flywheelServo.setPosition(0.5);
                sleep(500);
                flywheelServo.setPosition(0.9375);
                sleep(2000) ;
            }
                sleep(1000);
            stop() ;
            //moveWPID(48,48);
            //moveWPID(0,48);
            //moveWPID(0,-48);

            //moveWPID(-48,0);

            //moveWPID(-24,-48);
            //  }
            /*
            for (int i =0 ; i < 5 ; i = i + 1) {

                //backRightPower = SetRPM(targetRPM, backRightPower);
                moveWPID(48,4);
                moveWPID(0,-48);
                moveWPID(-24,0);

                //moveWPID(-24,-48);
            }
            */


        }
    }

    public double getRPM(double waitTime ){
        ElapsedTime timer = new ElapsedTime ();
        double startFWCount = flywheelShooter.getCurrentPosition();
        while (timer.milliseconds() < waitTime)
        {
        }

        double timeVar = (250.0/waitTime);
        double deltaFW = flywheelShooter.getCurrentPosition() - startFWCount;

        double RPM = timeVar * (deltaFW * 240)/537.6;

        return RPM;

    }


    public double SetRPM (double targetRPM, double motorPower){

        double time_step = 50.0 ;

        double time_step_mul = time_step / 50.0 ;

        double kp = 0.0025  * 1 ;
        double ki = (0.0025/50.0) * 0.1 * 1 ;
        double kd = 0.00025  * 1 ;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double errorRPM = targetRPM + getRPM(time_step);
        double curPower = motorPower;
        double lastErr = 0 ;
        double integralErr = 0 ;
        int inLockCount = 0 ;
        int loop_count = 0 ;
        while (loop_count < 1000) {
            double deltaError = errorRPM - lastErr;
            lastErr = errorRPM ;
            double time_int = timer.time() ;
            timer.reset();

            double derivative =  deltaError/time_int ;


            if (Math.abs(errorRPM) < 5 ) {
                integralErr += errorRPM * time_int;
            } else {
                integralErr += 0 ;
//                integralErr += ((errorRPM > 0) ? 5 * time_int : -5 * time_int) ;
            }

            double deltaPower = -1 * time_step_mul * ((errorRPM * kp) + (integralErr * ki) +(derivative * kd)) ;

            double pwrMul = (Math.abs(errorRPM) > 20) ? 1.0 :
                            (Math.abs(errorRPM) > 10)  ? 1.0/4.0 :
                            (Math.abs(errorRPM) > 5)  ? 1.0/16.0 :
                                    (Math.abs(errorRPM) > 2.5)  ? 01.0/64.0 : (1.0/128.0) ;
            curPower += (deltaPower * pwrMul) ;

            if (curPower > 0.7) curPower = 0.7 ;
            if (curPower < -0.7) curPower = -0.7 ;

            flywheelShooter.setPower(curPower);
            double RPM = getRPM(time_step);
            errorRPM = targetRPM + RPM;
            telemetry.addData("RPM = ", RPM);
            telemetry.addData("errorRPM = ", errorRPM);
            telemetry.addData("curPower  = ", curPower);
            telemetry.addData("deltaPower  = ", deltaPower);
            telemetry.update();

            if (Math.abs(errorRPM) <  2.5 ){
                inLockCount += 1 ;
                if (inLockCount > 20) {
                    return (curPower);
                }
            }
            else {
                inLockCount = 0 ;
            }
        }
        return (curPower);
    }

    public double SetRPMWobbleGoal(double targetRPM, double motorPower) {
        double time_step = 50.0 ;
        double kp = 0.0025;
        double ki = 0.0000025 * 0;
        double kd = 0.00000005 * 0;
        double errorRPM = targetRPM + getRPM(time_step);
        double curPower = motorPower;
        double lastErr = 0;
        double integralErr = 0;
        ElapsedTime timer = new ElapsedTime();

        while (Math.abs(errorRPM) > 1) {
            double deltaError = errorRPM - lastErr;
            if (Math.abs(errorRPM) < 15) integralErr += errorRPM * timer.time();
            double derivative = deltaError / timer.time();

            timer.reset();

            double deltaPower = -((errorRPM * kp) + (integralErr * ki) + (derivative * kd));

            curPower += deltaPower;

            if (curPower > 0.7) curPower = 0.7;
            if (curPower < -0.7) curPower = -0.7;

            flywheelShooter.setPower(curPower);
            double RPM = getRPM(time_step);
            errorRPM = targetRPM + RPM;
            telemetry.addData("RPM = ", RPM);
            telemetry.addData("errorRPM = ", errorRPM);
            telemetry.addData("curPower  = ", curPower);
            telemetry.addData("deltaPower  = ", deltaPower);
            telemetry.update();

            if (Math.abs(errorRPM) < 0.5) {
                return (curPower);
            }
        }
        return (curPower);
    }
    /*
    public void initDriveMotors(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
    public void moveWPID (double targetXInches, double targetYInches){
        double targetXCount = targetXInches * COUNTS_PER_INCH;
        double targetYCount = targetYInches * COUNTS_PER_INCH;
        // get starting X and Y position from encoders
        // and solving from equation

        double initialYPos = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition())/4;
        double initialXPos = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition() - backLeft.getCurrentPosition() - frontRight.getCurrentPosition())/4;
        // adding Count + initial
        double targetXPos = targetXCount + initialXPos;
        double targetYPos = targetYCount + initialYPos;
        // setting up X and Y for loop change
        double currentXPos = initialXPos;
        double currentYPos = initialYPos;
        double kp = 0.000005;
        double ki = 0.000025;
        double kd = 0.00005;
        double integralX = 0;
        double integralY = 0;
        double finalGain = 4 ;

        double errorX = targetXPos - currentXPos;
        double errorY = targetYPos - currentYPos;

        boolean movementDoneX = (Math.abs(errorX)<25) ;
        boolean movementDoneY = (Math.abs(errorY)<25) ;

        double lastXError = 0;
        double lastYError = 0;
        double curPowerX = 0;
        double curPowerY = 0;
        double capPowerX = .35;
        double capPowerY = .35;
        double deltaKX = 1;
        double deltaKY = 1;
        boolean firstPass = true ;


        double curPowerLF = 0;
        double curPowerLB = 0;
        double curPowerRF = 0;
        double curPowerRB = 0;

        ElapsedTime timer = new ElapsedTime();
        // start loop while any error is > some number
        while ((!movementDoneX || !movementDoneY) ){

            double deltaXError = firstPass ? 0 : errorX - lastXError;
            double deltaYError = firstPass ? 0 : errorY - lastYError;

            firstPass = false ;

            double curTime = timer.time();

            integralX += errorX * curTime;
            integralY += errorY * curTime;

            double derivativeX = deltaXError/curTime;
            double derivativeY = deltaYError/curTime;


            timer.reset();

            if (movementDoneX) deltaKX = 0;
            if (movementDoneY) deltaKY = 0;

            double deltaXPower = deltaKX * ((errorX * kp) + (integralX * ki) + (derivativeX * kd));
            double deltaYPower = deltaKY * ((errorY * kp) + (integralY * ki) + (derivativeY * kd));

            curPowerX = finalGain * deltaXPower;
            curPowerY = finalGain * deltaYPower;

            double usePwrX = curPowerX ;
            double usePwrY = curPowerY ;

            if (curPowerX > capPowerX) usePwrX = capPowerX;
            if (curPowerX < (-1 *capPowerX)) usePwrX = -1 * capPowerX;

            if (curPowerY > capPowerY) usePwrY = capPowerY;

            if (curPowerY < (-1 * capPowerY)) usePwrY = -1 * capPowerY;

            double PwrRatioX = (curPowerX != 0) ? Math.abs(usePwrX/curPowerX) : 0  ;
            double PwrRatioY = (curPowerY != 0) ? Math.abs(usePwrY/curPowerY) : 0 ;

            if (PwrRatioX != PwrRatioY) {
                if ((PwrRatioX != 0) && (PwrRatioX < PwrRatioY)) {
                    usePwrY = PwrRatioX * usePwrY / PwrRatioY  ;
                }
                if ((PwrRatioY != 0) && (PwrRatioY < PwrRatioX)) {
                    usePwrX = PwrRatioY * usePwrX / PwrRatioX  ;
                }

            }

            usePwrX = (!movementDoneX) ? usePwrX : 0 ;
            usePwrY = (!movementDoneY) ? usePwrY : 0 ;

            curPowerLF = usePwrY + usePwrX;
            curPowerLB = usePwrY - usePwrX;
            curPowerRF = usePwrY - usePwrX;
            curPowerRB = usePwrY + usePwrX;

            backLeft.setPower(curPowerLB);
            frontRight.setPower(curPowerRF);

            frontLeft.setPower(curPowerLF);
            backRight.setPower(curPowerRB);

            sleep(10);
            currentYPos = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition())/4;
            currentXPos = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition() - backLeft.getCurrentPosition() - frontRight.getCurrentPosition())/4;
            errorX = (targetXPos - currentXPos) ;
            errorY = (targetYPos - currentYPos);


            lastXError = errorX;
            lastYError = errorY;

            movementDoneX = (Math.abs(errorX)<25) || movementDoneX;
            movementDoneY = (Math.abs(errorY)<25) || movementDoneY;

            telemetry.addData("ErrX = ", errorX) ;
            telemetry.addData("ErrY = ", errorY) ;
            telemetry.addData("Front Left Encoder =", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Encoder ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder", backLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder =" , backRight.getCurrentPosition());
            telemetry.addData("Power ratio x =" , PwrRatioX);
            telemetry.addData("Power ratio y =" , PwrRatioY);



            telemetry.update();


        }

        double rampMul = 1.0 ;

        for (int i =0 ; i < 20 ; i++) {
            frontLeft.setPower(curPowerLF * rampMul );
            frontRight.setPower(curPowerRF *rampMul );

            backRight.setPower(curPowerRB * rampMul);
            backLeft.setPower(curPowerLB * rampMul);

            rampMul -= 0.05 ;

            sleep(10);
        }


    }
    */

    protected void initHardware() {
        // Vuforia and Tensorflow related initialization
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        /*
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

         */

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
//        robot.init(hardwareMap);
        /*
         */
        /*
        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "right_back");
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        */


        flywheelShooter = hardwareMap.get(DcMotor.class, "flywheel_shooter");
        flywheelShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flywheelShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelServo = hardwareMap.get(Servo.class, "flywheel_servo");

/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new StarterStackDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

 */
    /*
        telemetry.addData("Status", "Initialization Done");
        telemetry.update();
    }


     */
    }
}

