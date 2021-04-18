package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="Auto Ultimate Goal Base", group="PiRhos")
//@Disabled
@Disabled
public abstract class UltimateGoalAutonomousBase extends LinearOpMode {

    /* Declare OpMode members. */
//    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    protected DcMotor frontLeft, frontRight, backLeft, backRight, FW;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, rotation;
    PIDController           pidRotate, pidDrive;

    protected ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 / 3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;   // For figuring circumference - 100mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_SLOW = 0.4;
    static final double DRIVE_SPEED = 0.7;


    // Vuforia and Tensorflow related initialization
    /*private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    protected static final String LABEL_FIRST_ELEMENT = "Quad";
    protected static final String LABEL_SECOND_ELEMENT = "Single";
    protected static final String LABEL_NONE_ELEMENT = "None";

    private static final String VUFORIA_KEY =
            "AdK8eN7/////AAABmW0I+yjROEzugc7U5K8Gc50Zsbe0yWcOgrl6WKqYU/Fonb8sBLtyq4sTsoBNG9FeQqCrpmDZnDIXbsXXTUAxWGAfB4nRg+5+qjg8K+zwZGJWgEFxUonjXaeC6dCfnoQ9ZcCi6im+BkmEw5g3uXVdYr8J+6ygmRLd5LGIq4p3huYdrq3JQWNu43Vuwz5Bb9is861q7XQ224ZXvpMzHdU3CcvLo8imHVbFPUOVaV6cquuEyqcFArHhNUWn4m3IrrDaLRzCkD3dK5fT1+WF43BTq2C8NmtGYDC6v8p3e7+NTPUKzxhJiTZk7yxNAYiNd8U+vMdKi1Dxe3OFCKTRT+xiLjW6arbQnj+vmIZlZwfcIZ7o";
    */
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //protected VuforiaLocalizer vuforia;

    /**
     *  is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    //protected TFObjectDetector tfod;

    //OpenCV related initalization..
//    OpenCvInternalCamera phoneCam;
//    OpenCvCamera webcam;
    StarterStackDeterminationPipeline pipeline;

    protected void initHardware() {
        // Vuforia and Tensorflow related initialization
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();
        //initTfod();

        //if (tfod != null) {
       //     tfod.activate();
        //}

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
//        robot.init(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "right_back");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        pidRotate = new PIDController( .003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);

        pipeline = new StarterStackDeterminationPipeline();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        phoneCam.setPipeline(pipeline);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
/*
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
*/
        telemetry.addData("Status", "Initialization Done");
        telemetry.update();
    }


    protected void moveSideway(double speed, int leftPos, int rightPos) {

        // Right = +ve speed; Left = -ve speed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setTargetPosition(leftPos);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition(rightPos);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy())) {
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

    }

    protected void moveFwdAndBack(double speed, int leftPos, int rightPos) {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int leftTargetCount = (int)(COUNTS_PER_INCH * leftPos);
        int rightTargetCount = (int)(COUNTS_PER_INCH * rightPos);


        frontLeft.setTargetPosition(leftTargetCount);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition(rightTargetCount);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Speed: ", speed);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

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

    }


    protected void moveForwardandBackwardEncoders( double speed,
                                                   double distance ,double timeoutInMilliseconds){
        int newBackLeftTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);
            backLeft.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.milliseconds() < timeoutInMilliseconds) &&
                    (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    protected void TurnClockWiseAndAntiClockwise( double speed,
                                                   double rightdistance, double leftdistance,double timeoutInMilliseconds){
        int newBackLeftTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftdistance * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(rightdistance * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(rightdistance * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(leftdistance * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);
            backLeft.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.milliseconds() < timeoutInMilliseconds) &&
                    (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    protected void moveSideWaysEncoders( double speed, double distance ,double timeoutInMilliseconds) {
        speed = -speed;
        int newBackLeftTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.milliseconds() < timeoutInMilliseconds) &&
                    (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

        public void TurnMecanumDriveWithAngle(double targetAngle, double translationPower, double turnPower)
        {
            resetAngle();
            // calculate motor power
            double ADPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(targetAngle) + Math.cos(targetAngle));
            double BCPower = translationPower * Math.sqrt(2) * 0.5 * (Math.sin(targetAngle) - Math.cos(targetAngle));

            // check if turning power will interfere with normal translation
            // check ADPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
            double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
            // check BCPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
            turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

            // adjust turn power scale correctly
            if (Math.abs(turningScale) < 1.0)
            {
                turningScale = 1.0;
            }

            // set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
/*
            frontLeft.setPower((ADPower - turningScale) / turningScale);
            backLeft.setPower((BCPower - turningScale) / turningScale);
            frontRight.setPower((BCPower + turningScale) / turningScale);
            backRight.setPower((ADPower + turningScale) / turningScale);
*/
            frontLeft.setPower(- turnPower);
            backLeft.setPower(- turnPower );
            frontRight.setPower(turnPower);
            backRight.setPower(turnPower);

            double currentAngle = getAngle();
            while(Math.abs(currentAngle - targetAngle) > 2.0){
                if ( currentAngle > targetAngle )
                    turnPower = -turnPower;

                if(Math.abs(currentAngle - targetAngle) < 15.0){
                    turnPower = turnPower * 0.3;
                    turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
                    // check BCPower to see if trying to apply turnPower would put motor power over 1.0 or under -1.0
                    turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

                    // adjust turn power scale correctly
                    if (Math.abs(turningScale) < 1.0)
                    {
                        turningScale = 1.0;
                    }

                    // set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
                    frontLeft.setPower(- turnPower);
                    backLeft.setPower(- turnPower );
                    frontRight.setPower(turnPower);
                    backRight.setPower(turnPower);

                }

                currentAngle = getAngle();
                telemetry.addData("Current Angle= ", currentAngle);
                telemetry.update();

                sleep(500);

            }
        }

    public void resetAngle()
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

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        telemetry.addData("Global Angle= ", globalAngle);
        telemetry.update();

        lastAngles = angles;

        return globalAngle;


    }

        public void simpleversionoftheturningmethod(double translationAngle2, double translationPower2, double turnPower2)
        {
            // calculate motor power
            double ADPower = translationPower2 * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle2) + Math.cos(translationAngle2));
            double BCPower = translationPower2 * Math.sqrt(2) * 0.5 * (Math.sin(translationAngle2) - Math.cos(translationAngle2));

            // set the motors
            frontLeft.setPower(ADPower);
            backLeft.setPower(BCPower);
            frontRight.setPower(BCPower);
            backRight.setPower(ADPower);
        }

        public void rotateusingPID(int degrees, double power)
        {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }















    protected void moveSidewayForDistance(double speed, double inches, double timeoutInMilliseconds) {
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

    }

    /*protected String TFRecognizeStack(double milliseconds) {
        ElapsedTime runtime = new ElapsedTime();
        String stackHeight = LABEL_NONE_ELEMENT;
        while (opModeIsActive() && runtime.milliseconds() < milliseconds) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        stackHeight = recognition.getLabel();
                        break;
                    }
                    telemetry.update();
                }
            }
            if ( !stackHeight.equalsIgnoreCase(LABEL_NONE_ELEMENT) )
                break;

        }
        return stackHeight;
    }*/

    protected StarterStackDeterminationPipeline.RingPosition OpenCVRecognizeStack(double milliseconds) {
        ElapsedTime runtime = new ElapsedTime();
        StarterStackDeterminationPipeline.RingPosition stackHeight = StarterStackDeterminationPipeline.RingPosition.NONE;

        while (opModeIsActive() && runtime.milliseconds() < milliseconds)
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData( "Position", pipeline.position);
            telemetry.update();
            stackHeight = pipeline.getAnalysis();
            if (!stackHeight.equals(StarterStackDeterminationPipeline.RingPosition.NONE))
                break;

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        return stackHeight;
    }

    // Vuforia and Tensorflow related functions

    /**
     * Initialize the Vuforia localization engine.
     */
   // private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
       // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       // parameters.vuforiaLicenseKey = VUFORIA_KEY;
       // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    //}

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    /*private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }*/

    public static class StarterStackDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200,50);
        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile StarterStackDeterminationPipeline.RingPosition position = StarterStackDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = StarterStackDeterminationPipeline.RingPosition.FOUR; // Record our analysis

            if ( avg1 > FOUR_RING_THRESHOLD )
                position = StarterStackDeterminationPipeline.RingPosition.FOUR;
            else if ( avg1 > ONE_RING_THRESHOLD )
                position = StarterStackDeterminationPipeline.RingPosition.ONE;
            else
                position = StarterStackDeterminationPipeline.RingPosition.NONE;

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public StarterStackDeterminationPipeline.RingPosition getAnalysis()
        {
            return position;
        }
    }

}
