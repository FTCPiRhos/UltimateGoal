package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Auto Ultimate Goal Base OpenCV", group="PiRhos")
//@Disabled
public abstract class UltimateGoalAutonomousBaseOpenCV extends LinearOpMode {

    /* Declare OpMode members. */
    protected DcMotor frontLeft, frontRight, backLeft, backRight;

    protected ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440 / 2;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 / 3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;   // For figuring circumference - 100mm
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_SLOW = 0.4;
    static final double DRIVE_SPEED = 0.7;

    //OpenCV related initalization
    OpenCvInternalCamera phoneCam;
    StarterStackDeterminationPipeline pipeline;

    protected void initHardware() {

        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "rightRear");
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new StarterStackDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

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

        telemetry.addData("Status", "Initialization Done");
        telemetry.update();
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
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Speed", speed);
            telemetry.addData("Second", milliseconds);
            telemetry.update();
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

}