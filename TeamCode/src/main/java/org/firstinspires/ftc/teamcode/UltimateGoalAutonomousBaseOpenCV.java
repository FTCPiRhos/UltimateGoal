package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.vision.OpenCVCrCb;
import org.firstinspires.ftc.teamcode.vision.StarterStackDeterminationExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@Autonomous(name="Auto Ultimate Goal Base OpenCV", group="PiRhos")
//@Disabled
public abstract class UltimateGoalAutonomousBaseOpenCV extends LinearOpMode {

    /* Declare OpMode members. */
    protected DcMotor frontLeft, frontRight, backLeft, backRight, flywheelShooter, armMotor;
    protected Servo armServo, flywheelServo;

    protected ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 / 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;   // For figuring circumference - 100mm
    static final double COUNTS_PER_INCH = 1.45 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_SLOW = 0.4;
    static final double DRIVE_SPEED = 0.7;

    //OpenCV related initalization
    OpenCvInternalCamera webcam;
    StarterStackDeterminationPipeline old_pipeline;
    OpenCVTestPipeline pipeline;

    //Rotation related
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    PIDController           pidRotate, pidDrive;
    GyroSensor gyro;
    Orientation angles;
    Acceleration gravity;

    protected void initHardware( boolean fOpenCVLeft ) {

        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "right_back");
        armServo = hardwareMap.get(Servo.class,"arm_servo");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flywheelShooter = hardwareMap.get(DcMotor.class, "flywheel_shooter");
        flywheelShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelServo = hardwareMap.get(Servo.class, "flywheel_servo");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection((DcMotor.Direction.REVERSE));
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //Rotation related
        pidRotate = new PIDController( .003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        old_pipeline = new StarterStackDeterminationPipeline( fOpenCVLeft );
        pipeline = new OpenCVTestPipeline( fOpenCVLeft );
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
           @Override
           public void onOpened()
           {
               webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        telemetry.addData("Status", "Initialization Done");
        telemetry.update();
        sleep(2000);
        telemetry.addData("Status", "Ready");
        telemetry.update();
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

        double time_step = 100.0 ;

        double time_step_mul = time_step / 50.0 ;

        double kp = 0.0025  * 1 ;
        double ki = (0.0025/50.0) * 0.1 * 1 ;
        double kd = 0.0005  * 1;

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

            /* double pwrMul = (Math.abs(errorRPM) > 20) ? 1.0 :
                            (Math.abs(errorRPM) > 10)  ? 1.0/4.0 :
                            (Math.abs(errorRPM) > 5)  ? 1.0/16.0 :
                                    (Math.abs(errorRPM) > 2.5)  ? 01.0/64.0 : (1.0/128.0) ;

             */
            double pwrMul = 1.0;
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

            if (Math.abs(errorRPM) <  2 ){
                inLockCount += 1 ;
                if (inLockCount > 12) {
                    return (curPower);
                }
            }
            else {
                inLockCount = 0 ;
            }
        }
        return (curPower);
    }


    public void shooterTrigger (){
            flywheelServo.setPosition(0.5);
            sleep(500);
            flywheelServo.setPosition(1);
            //sleep(0) ;

    }
    public double SetRPMWobbleGoal (double targetRPM, double motorPower){
        double kp = 0.0025;
        double ki = 0.0000025 * 0;
        double kd = 0.00000005 * 0 ;
        double errorRPM = targetRPM + getRPM(50);
        double curPower = motorPower;
        double lastErr = 0 ;
        double integralErr = 0 ;
        ElapsedTime timer = new ElapsedTime();

        while (Math.abs(errorRPM) > 1) {
            double deltaError = errorRPM - lastErr;
            if (Math.abs(errorRPM)<15) integralErr += errorRPM * timer.time();
            double derivative = deltaError/timer.time();

            timer.reset();

            double deltaPower = - ((errorRPM * kp) + (integralErr * ki) +(derivative *kd)) ;

            curPower += deltaPower ;

            if (curPower > 0.7) curPower = 0.7 ;
            if (curPower < -0.7) curPower = -0.7 ;

            flywheelShooter.setPower(curPower);
            double RPM = getRPM(50);
            errorRPM = targetRPM + RPM;
            telemetry.addData("RPM = ", RPM);
            telemetry.addData("errorRPM = ", errorRPM);
            telemetry.addData("curPower  = ", curPower);
            telemetry.addData("deltaPower  = ", deltaPower);
            telemetry.update();

            if (Math.abs(errorRPM) < 1) {
                return (curPower);
            }
        }
        return (curPower);
    }

    public void moveWPID (double targetXInches, double targetYInches){


        frontLeft.setPower(0);
        frontRight.setPower(0);

        backRight.setPower(0);
        backLeft.setPower(0);


        double targetXCount = targetXInches * COUNTS_PER_INCH;
        double targetYCount = targetYInches * COUNTS_PER_INCH;
        // get starting X and Y position from encoders
        // and solving from equation

        double initialYPos = ( backLeft.getCurrentPosition() + backRight.getCurrentPosition())/2;
        double initialXPos = ( backRight.getCurrentPosition() - backLeft.getCurrentPosition())/2;
        // adding Count + initial
        double targetXPos = targetXCount + initialXPos;
        double targetYPos = targetYCount + initialYPos;
        // setting up X and Y for loop change
        double currentXPos = initialXPos;
        double currentYPos = initialYPos;
        double kp = 0.000005;
        double ki = 0.00005;
        double kd = 0.00005;
        double integralX = 0;
        double integralY = 0;
        double finalGain = 5 ;

        double errorX = targetXPos - currentXPos;
        double errorY = targetYPos - currentYPos;

        boolean movementDoneX = (Math.abs(errorX)<25) ;
        boolean movementDoneY = (Math.abs(errorY)<25) ;

        double lastXError = 0;
        double lastYError = 0;
        double curPowerX = 0;
        double curPowerY = 0;
        double capPowerX = .75;
        double capPowerY = .75;
        double minPowerX = 0;
        double minPowerY = 0;
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
            double powerLowThreshMul = 0;

            if (((Math.abs(curPowerX)) > minPowerX )  ||  ((Math.abs(curPowerY)) > minPowerY)) powerLowThreshMul = 1;

            double usePwrX = powerLowThreshMul * curPowerX ;
            double usePwrY = powerLowThreshMul * curPowerY ;

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

            backRight.setPower(curPowerRB);
            backLeft.setPower(curPowerLB);

            frontLeft.setPower(curPowerLF);
            frontRight.setPower(curPowerRF);

            sleep(50);

            double posBL = backLeft.getCurrentPosition() ;
            double posBR = backRight.getCurrentPosition() ;

            double posFL = frontLeft.getCurrentPosition() ;
            double posFR = frontRight.getCurrentPosition() ;

            currentYPos = (posBL  + posBR)/2;
            currentXPos = ( posBR - posBL)/2;

            errorX = (targetXPos - currentXPos) ;
            errorY = (targetYPos - currentYPos);


            lastXError = errorX;
            lastYError = errorY;

            movementDoneX = (Math.abs(errorX)<100) || movementDoneX;
            movementDoneY = (Math.abs(errorY)<100) || movementDoneY;

            telemetry.addData("ErrX = ", errorX) ;
            telemetry.addData("ErrY = ", errorY) ;
            telemetry.addData("Front Left Encoder =", posFL);
            telemetry.addData("Front Right Encoder ", posFR);
            telemetry.addData("Back Left Encoder", posBL);
            telemetry.addData("Back Right Encoder =" , posBR);
            telemetry.addData("Power ratio x =" , PwrRatioX);
            telemetry.addData("Power ratio y =" , PwrRatioY);



            telemetry.update();


        }

        double rampMul = 1.0 ;

        for (int i =0 ; i < 5 ; i++) {
            rampMul -= 0.2 ;
            frontLeft.setPower(curPowerLF * rampMul );
            frontRight.setPower(curPowerRF *rampMul );

            backRight.setPower(curPowerRB * rampMul);
            backLeft.setPower(curPowerLB * rampMul);
            sleep(10);
        }


    }

    protected void moveFwdAndBackForMilliseconds(double speed, double milliseconds) {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && runtime.milliseconds() < milliseconds) {
            // Display it for the driver.
            //telemetry.addData("LF", frontLeft.getCurrentPosition());
            //telemetry.addData("RF", frontRight.getCurrentPosition());
            //telemetry.addData("LB", backLeft.getCurrentPosition());
            //telemetry.addData("RB", backRight.getCurrentPosition());
            //telemetry.addData("Speed", speed);
            //telemetry.addData("Second", milliseconds);
            //telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveSidewayForMilliseconds(double speed, double milliseconds) {
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && runtime.milliseconds() < milliseconds) {
            // Display it for the driver.
            //telemetry.addData("LF", frontLeft.getCurrentPosition());
            //telemetry.addData("RF", frontRight.getCurrentPosition());
            //telemetry.addData("LB", backLeft.getCurrentPosition());
            //telemetry.addData("RB", backRight.getCurrentPosition());
            //telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveFwdAndBackForDistance(double speed, double inches, double timeoutInMilliseconds) {
/*
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        ElapsedTime runtime = new ElapsedTime();
        double startPostion = frontLeft.getCurrentPosition();
        //COUNTS_PER_INCH
        while (opModeIsActive() && runtime.milliseconds() < timeoutInMilliseconds) {

            double leftDiff = startPostion + inches * COUNTS_PER_INCH + frontLeft.getCurrentPosition();
            double slowDownFactor = 1.0;
            if (leftDiff < 2 * COUNTS_PER_INCH) {
                slowDownFactor = (double) leftDiff / (2 * COUNTS_PER_INCH);
            }

            if (Math.abs(frontLeft.getCurrentPosition() - startPostion) < 50)
                break;


            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
*/
    }

    protected void moveSidewayForDistance(double speed, double inches, double timeoutInMilliseconds) {
/*
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        ElapsedTime runtime = new ElapsedTime();
        double startPostion = backLeft.getCurrentPosition();

        while (opModeIsActive() && runtime.milliseconds() < timeoutInMilliseconds) {

            double leftDiff = startPostion + inches * COUNTS_PER_INCH + backLeft.getCurrentPosition();
            double slowDownFactor = 1.0;
            if (leftDiff < 2 * COUNTS_PER_INCH) {
                slowDownFactor = (double) leftDiff / (2 * COUNTS_PER_INCH);
            }

            if (Math.abs(frontLeft.getCurrentPosition() - startPostion) < 50)
                break;


            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
*/
    }

    protected void rotate( double speed, int degree ) {

    }

    // Shooting ring with flywheel
    protected void shoot() {

    }

    protected OpenCVTestPipeline.RingPosition OpenCVRecognizeStack(double milliseconds ) {
        ElapsedTime runtime = new ElapsedTime();
        OpenCVTestPipeline.RingPosition stackHeight = OpenCVTestPipeline.RingPosition.NONE;

        while (opModeIsActive() && runtime.milliseconds() < milliseconds)
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Ratio", pipeline.ratio);
            telemetry.update();
            stackHeight = pipeline.getAnalysis();
            if (!stackHeight.equals(OpenCVTestPipeline.RingPosition.NONE))
                break;

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        return stackHeight;
    }
    protected void ArmEncoders(double speed, double distance, int timeoutInMilliseconds) {
        int newArmTarget;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newArmTarget = armMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            armMotor.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            armMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.milliseconds() < timeoutInMilliseconds) &&
                    (armMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newArmTarget);
                telemetry.addData("Path2", "Running at %7d", armMotor.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            armMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

        protected void CommonMethodForArm() {

            ArmEncoders(0.7, -0.88, 10000);
            armServo.setPosition(0);
            moveWPID(8, 0);
            ArmEncoders(0.7, 1.3, 10000);
            armServo.setPosition(1);


        }
    public void shooterTrigger3x (){
        for (int i = 0 ; i < 3 ; i += 1) {
            double targetRPM = -127 ;
            double flywheelPower = 0.47;
            flywheelPower = SetRPM(targetRPM, flywheelPower);
            flywheelPower = 1.0 * flywheelPower;
            flywheelServo.setPosition(0.5);
            sleep(500);
            flywheelServo.setPosition(1);
            //sleep(0) ;
        }
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaFirstAngle = angles.firstAngle - lastAngles.firstAngle;




        if (deltaFirstAngle < -180)
            deltaFirstAngle += 360;
        else if (deltaFirstAngle > 180)
            deltaFirstAngle -= 360;

        globalAngle += deltaFirstAngle;

        lastAngles = angles;

        telemetry.addData("First Angle = ", angles.firstAngle);
        telemetry.addData("Second Angle = ", angles.secondAngle);
        telemetry.addData("Third Angle = ", angles.thirdAngle);
        telemetry.update();




        return globalAngle;




    }
    public boolean rotate(int degrees, double power)
    {

        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return false;

        // set power to rotate.
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

/*
        for ( int i = 0; i < 100; i++ ) {
            telemetry.addLine(">>> " + getAngle());
            telemetry.update();
            sleep(1000);
        }
        // rotate until turn is completed.
        // On right turn we have to get off zero first.
*/
        degrees = -degrees;
        while (opModeIsActive() && Math.abs( getAngle() - degrees ) > 5.0 ) {
            sleep(10);
            // if( rotated 60 percent), reduce the speed of the wheels by half.

            double AnglePrecToSlowDown = 0.8;
            if ( degrees <= 60.0 )
                AnglePrecToSlowDown =0.6;

            if(getAngle() > AnglePrecToSlowDown * degrees ) {
                double diff = degrees - getAngle();
                double modifier = 0.2 + 0.3 * ( diff/ degrees );
                frontRight.setPower(rightPower * modifier );
                backLeft.setPower(leftPower * modifier);
                frontLeft.setPower(leftPower * modifier);
                backRight.setPower(rightPower * modifier);



            }



        }

        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);


/*
        while (opModeIsActive() && getAngle() > degrees) {
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            frontLeft.setPower(-power);
            backRight.setPower(-power);
        }
*/

        // wait for rotation to stop.
        sleep(200);

        // reset angle tracking on new heading.
        resetAngle();
        return false;
    }

        }




