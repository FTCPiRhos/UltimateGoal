package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name="Current Teleop OpenCV.", group="PiRhos")





public class OpenCVTeleop extends LinearOpMode {
    class SetRPMVars {
        ElapsedTime timer = new ElapsedTime();
        boolean isValid = false;
        boolean inWhile = false ;
        boolean isPowershot = false;
        double pwrMul = 1.0;
        double curPower;
        double errorRPM;
        double curTime;
        double deltaError;

        double time_step = 25.0;

        double time_step_mul = time_step / 50.0;

        double kp = 0.0025 * 1;
        double ki = (0.0025 / 50.0) * 0.1 * 1;
        double kd = 0.0005 * 1;


        double lastErr = 0;
        double integralErr = 0;
        int inLockCount = 0;
        int loop_count = 0;
    }

    private SetRPMVars shooterRPMVars = new SetRPMVars();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor armMotor = null;
    private Servo armServo = null;
    private Servo flywheelServo = null;
    private DcMotor flywheelShooter = null;
    private DcMotor intakeTop = null;
    private DcMotor intakeBottom = null;
    private Servo ringBlockerRight = null;
    private Servo ringBlockerLeft = null;
    private CRServo frontIntakeFront = null;
    private CRServo frontIntakeBack = null;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 / 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;   // For figuring circumference - 100mm
    static final double COUNTS_PER_INCH = 1.45 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_SLOW = 0.4;
    static final double DRIVE_SPEED = 0.7;
    double intakeBottomShooterPwr = 0.7;
    double intakeBottomPwr = -0.95;
    double intakeTopPwr = 0.95;
    double shooterServoRestPos = 0.6;
    double shooterServoFlickPos = 1.0;
    double calibPwr;
    double calibMult = 1.0;
    double rightBlockerRestPos = 0.62;
    double leftBlockerRestPos = 0.935;
    double rightBlockerBlockingPos = 1;
    double leftBlockerBlockingPos = 0.635;
    boolean blockersDown = false;
    static final double YR10x = 674876.0;
    static final double YL10x = -650615.0;
    static final double COUNTS_PER_DEGREE_ODO = ((((YR10x - YL10x)/10)/360) * 0.995);
    OpenCvInternalCamera webcam;
    StarterStackDeterminationPipeline old_pipeline;
    OpenCVPipelineShoot pipeline;
    double targetRPMGoal = -167;

    // 1 is parallel on right



    // servo is at port 0 of main
    // motor is at port 3 of secondary


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        frontLeft = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");
        backLeft = hardwareMap.get(DcMotor.class, "left_back");
        backRight = hardwareMap.get(DcMotor.class, "right_back");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armServo = hardwareMap.get(Servo.class, "arm_servo");
        ringBlockerLeft = hardwareMap.get(Servo.class,"right_blocker");
        ringBlockerRight = hardwareMap.get(Servo.class,"left_blocker");
        frontIntakeFront = hardwareMap.get(CRServo.class,"front1");
        frontIntakeBack = hardwareMap.get(CRServo.class, "front2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection((DcMotor.Direction.REVERSE));
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        ringBlockerLeft.setDirection(Servo.Direction.FORWARD);
        ringBlockerRight.setDirection(Servo.Direction.FORWARD);
        frontIntakeFront.setDirection(CRServo.Direction.FORWARD);
        frontIntakeBack.setDirection(CRServo.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);

        intakeTop = hardwareMap.get(DcMotor.class, "intake2");
        intakeBottom = hardwareMap.get(DcMotor.class, "intake1");
        intakeTop.setDirection(DcMotor.Direction.FORWARD);
        intakeBottom.setDirection(DcMotor.Direction.REVERSE);

        flywheelShooter = hardwareMap.get(DcMotor.class, "flywheel_shooter");
        flywheelShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flywheelShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelServo = hardwareMap.get(Servo.class, "flywheel_servo");
        flywheelServo.setPosition(shooterServoRestPos);
        armServo.setPosition(1);
        ringBlockerLeft.setPosition(leftBlockerRestPos);
        ringBlockerRight.setPosition(rightBlockerRestPos);

        // init cam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        old_pipeline = new StarterStackDeterminationPipeline(true);
        pipeline = new OpenCVPipelineShoot(false);
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean sens = true;
        double sensMult = 1.0;

        boolean IntakeCalibrated = false;
        double targetRPMIntake = 150;
        double flywheelPower = 0.6;
        double maxSwitchPwr = 1.0;
        double LFPower;
        double LBPower;
        double RFPower;
        double RBPower;
        double ArmPower;
        double LFDrivePwrMul = 1;
        double RFDrivePwrMul = 1;
        double LBDrivePwrMul = 1;
        double RBDrivePwrMul = 1;
        double DrivePwrMul;
        double DrivePwrMulTrigger;
        double DrivePwrMulTriggerFast;
        double intakePower;
        double PowershotPower = 0;
        boolean firstPS = true;
        // target speed for shooter
        boolean firstGoalShot = true;
        double oldLFPos = 0;
        double oldLBPos = 0;
        double oldRFPos = 0;
        double oldRBPos  = 0;
        double LFPos ;
        double LBPos ;
        double RFPos ;
        double RBPos ;
        boolean LFflip = false;
        boolean LBflip = false;
        boolean RFflip = false;
        boolean RBflip = false;
        boolean shootOverride = false;


        //boolean drivePwrController = false;

        while (opModeIsActive()) {
            if (firstGoalShot){
                shooterRPMVars.isValid=true;
                firstGoalShot = false;
            }

            if (!shooterRPMVars.isPowershot){
                SetRPM(targetRPMGoal,flywheelPower);
                flywheelPower = shooterRPMVars.curPower;
            }
            else if (shooterRPMVars.isPowershot){
                SetRPM(-157,flywheelPower);
            }

            // motor pos get
            LFPos = frontLeft.getCurrentPosition();
            LBPos = backLeft.getCurrentPosition();
            RFPos = frontRight.getCurrentPosition();
            RBPos = backRight.getCurrentPosition();



            // multipliers for power
            intakePower = 1.6 * flywheelPower;
            DrivePwrMulTrigger = 1.0 - (gamepad1.right_trigger);
            DrivePwrMulTriggerFast = 1.0;
            //DrivePwrMulTriggerFast = 1.0;

            calibPwr = flywheelPower;
            // init variables

            // flip control
            if (gamepad1.right_bumper) sens = false;
            if (gamepad1.left_bumper) sens = true;
            if (sens) {
                sensMult = 1.0;
            }
            if (sens == false) {
                sensMult = -1.0;
            }
            //init power vars
            double arm = gamepad2.right_stick_y;
            double y = -gamepad1.left_stick_y * sensMult;
            double x = gamepad1.left_stick_x * sensMult;
            double rx = gamepad1.right_stick_x * 1.2;


            // calculate motor powers
            LFPower = (y + x + rx);
            LBPower = (y - x + rx);
            RFPower = (y - x - rx);
            RBPower = (y + x - rx);
            ArmPower = arm * -1;
            // make sure none of the drive powers are too high
            if (Math.abs(LFPower) > 1 || Math.abs(LBPower) > 1 ||
                    Math.abs(RFPower) > 1 || Math.abs(RBPower) > 1) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(LFPower), Math.abs(LBPower));
                max = Math.max(Math.abs(max), Math.abs(RFPower));
                max = Math.max(Math.abs(max), Math.abs(RBPower));

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                LFPower /= max;
                LBPower /= max;
                RFPower /= max;
                RBPower /= max;
            }

            // check for flip
            if ((LFPos - oldLFPos > 0) && (LFPower < 0)) {
                if (LFPower == 0) {

                }
                else {
                    LFDrivePwrMul = Math.abs(maxSwitchPwr / LFPower);
                    telemetry.addData("LF flipped ", 0);
                }
            }

            if ((LFPos - oldLFPos < 0) && (LFPower > 0)) {
                if (LFPower == 0){

                }
                else {
                    LFDrivePwrMul = Math.abs(maxSwitchPwr / LFPower);

                    telemetry.addData("LF flipped ", 0);
                }
            }

            if ((LBPos - oldLBPos > 0) && (LBPower < 0)) {
                if (LBPower == 0 ){

                }
                else {
                    LBDrivePwrMul = Math.abs(maxSwitchPwr / LBPower);


                    telemetry.addData("LB flipped ", 0);
                }
            }

            if ((LBPos - oldLBPos < 0) && (LBPower > 0)) {
                if (LBPower == 0 ){

                }
                else {
                    LBDrivePwrMul = Math.abs(maxSwitchPwr / LBPower);

                    telemetry.addData("LB flipped ", 0);
                }
            }


            if ((RFPos - oldRFPos > 0) && (RFPower < 0)) {
                if (RFPower == 0){

                }
                else {
                    RFDrivePwrMul = Math.abs(maxSwitchPwr / RFPower);
                    telemetry.addData("RF flipped ", 0);
                }
            }

            if ((RFPos - oldRFPos < 0) && (RFPower > 0)) {
                if (RFPower == 0){

                }
                else {
                    RFDrivePwrMul = Math.abs(maxSwitchPwr / RFPower);
                    telemetry.addData("RF flipped ", 0);
                }
            }

            if ((RBPos - oldRBPos > 0) && (RBPower < 0)) {
                if (RBPower == 0){

                }
                else {
                    RBDrivePwrMul = Math.abs(maxSwitchPwr / RBPower);


                    telemetry.addData("RB flipped ", 0);
                }
            }

            if ((RBPos - oldRBPos < 0) && (RBPower > 0)) {
                if (RBPower == 0){

                }
                else {
                    RBDrivePwrMul = Math.abs(maxSwitchPwr / RBPower);

                    telemetry.addData("RB flipped ", 0);
                }
            }

            // calculate multiplier
            if (LBDrivePwrMul > 1) LBDrivePwrMul =1;
            if (LFDrivePwrMul > 1) LFDrivePwrMul =1;
            if (RBDrivePwrMul > 1) RBDrivePwrMul =1;
            if (RFDrivePwrMul > 1) RFDrivePwrMul =1;

            DrivePwrMul = Math.min(LBDrivePwrMul , LFDrivePwrMul);
            DrivePwrMul = Math.min(DrivePwrMul, RBDrivePwrMul);
            DrivePwrMul = Math.min(DrivePwrMul, RFDrivePwrMul);

            // set power calc
            LFPower = LFPower * DrivePwrMul * DrivePwrMulTrigger * DrivePwrMulTriggerFast;
            LBPower = LBPower * DrivePwrMul * DrivePwrMulTrigger * DrivePwrMulTriggerFast;
            RFPower = RFPower * DrivePwrMul * DrivePwrMulTrigger * DrivePwrMulTriggerFast;
            RBPower = RBPower * DrivePwrMul * DrivePwrMulTrigger * DrivePwrMulTriggerFast;



            // arm power override

            if (gamepad2.right_trigger>0.5){
                ArmPower = arm * -1;
            }
            else {
                // arm power control
                ArmPower = (armMotor.getCurrentPosition() > 1000 && (ArmPower >=0.1)) ? 0.1 :
                        (armMotor.getCurrentPosition() < 300 && (ArmPower < -0.10)) ? -0.1 : ArmPower ;
            }

            // reset init values on arm
            if (gamepad2.dpad_down) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            // set powers
            frontLeft.setPower(LFPower);
            backLeft.setPower(LBPower);
            frontRight.setPower(RFPower);
            backRight.setPower(RBPower);
            armMotor.setPower(ArmPower);

            // servo open
            if (gamepad2.right_bumper == true) armServo.setPosition(0.4);

            // servo close
            if (gamepad2.left_bumper == true) armServo.setPosition(1);

            if (gamepad1.a)  {
                intakeTop.setPower(0 * intakeTopPwr);
                intakeBottom.setPower(0 * intakeBottomPwr);
              //  frontIntakeBack.setPower(0);
               // frontIntakeFront.setPower(0);
                sleep(100);
            }

            if (gamepad2.dpad_up){
                if (!shootOverride){
                    shootOverride = true;
                    sleep(250);
                }
                else if (shootOverride){
                    shootOverride = false;
                    sleep(250);

                }
            }


            // shoot
            if (gamepad1.dpad_up == true) {
                boolean aligned = false;
                double BasePower = 0.0;
                double power = 0.3;
                double mult = 1;
                double count = 0;
                double exitRange = 5;
                double error = 0;
                double startError = 0;
                boolean exit = false;

                intakeTop.setPower(0);
                intakeBottom.setPower(0);
                if (!shootOverride) {
                    while (!aligned && !pipeline.noObject) {

                        if (gamepad2.dpad_up) {
                            aligned = true;
                            exit = true;
                        }
                        //telemetry.addData("R1P1 = ", pipeline.getR1TopLeft());
                        //telemetry.addData("R1P2 = ", pipeline.getR1BottomRight());
                        //telemetry.addData("R2P1 = ", pipeline.getR2TopLeft());
                        //telemetry.addData("R2P2 = ", pipeline.getR2BottomRight());
                        //telemetry.update();
                        double Dx1 = 160 - pipeline.R1BottomRight.x;
                        double Dx2 = pipeline.R2TopLeft.x - 160;

                        boolean oneObject = (Math.abs(pipeline.R1BottomRight.x - pipeline.R2TopLeft.x) < 50);


                        if ((Math.abs(Dx1 - Dx2)) > 100) mult = 0.00;
                        else mult = (0.25 * (100 - (Math.abs(Dx1 - Dx2))) / 100);

                        //mult = 0;


                        double turnPower = power - mult;
                        if (turnPower < 0.3) turnPower = 0.3;


                        if (startError == 0) startError = (Math.abs(Dx1 - Dx2));

                        if (startError < 20) turnPower = 0.15;

                        if (((Math.abs(Dx1 - Dx2)) < exitRange) && (oneObject == false)) {
                            aligned = true;
                            frontRight.setPower(0);
                            backRight.setPower(0);
                            frontLeft.setPower(0);
                            backLeft.setPower(0);
                        } else if (((Dx2 - Dx1) > 0) || (oneObject && (pipeline.getColor() == OpenCVPipelineShoot.Color.RED))) {
                            frontRight.setPower(-1 * BasePower - turnPower);
                            backRight.setPower(BasePower - turnPower);
                            frontLeft.setPower(-1 * BasePower + turnPower);
                            backLeft.setPower(BasePower + turnPower);


                        } else {
                            frontRight.setPower(-1 * BasePower + turnPower);
                            backRight.setPower(BasePower + turnPower);
                            frontLeft.setPower(-1 * BasePower - turnPower);
                            backLeft.setPower(BasePower - turnPower);

                        }


                        sleep(15);

                        frontRight.setPower(0);
                        backRight.setPower(0);
                        frontLeft.setPower(0);
                        backLeft.setPower(0);
                        sleep(45);


                        count++;
                        exitRange += 2;
                        if (exitRange > 12) exitRange = 12;

                        error = (Math.abs(Dx1 - Dx2));
                    }

                    if (!exit && (pipeline.noObject == false)) {
                        if (!shooterRPMVars.isPowershot) {

                            for (int i = 0; i < 10; i++) {
                                SetRPM(targetRPMGoal, flywheelPower);
                                flywheelPower = shooterRPMVars.curPower;
                                sleep(10);
                            }


                            shoot3times(flywheelPower);
                        } else {
                            for (int i = 0; i < 10; i++) {
                                SetRPM(-150, flywheelPower);
                                flywheelPower = shooterRPMVars.curPower;
                                sleep(10);


                            }
                            double degreeSign = 1.0;
                            double degreeAdd = 0;
                            if (pipeline.getColor() == OpenCVPipelineShoot.Color.RED) {
                                degreeSign = -1.0;
                                degreeAdd = -3.0;
                            }
                            rotate(13 * degreeSign - degreeAdd, 0.4);
                            for (int i = 0; i < 3; i++) {

                                flywheelServo.setPosition(shooterServoFlickPos);
                                for (int j = 0; j < 20; j++) {
                                    SetRPM(-150, flywheelPower);
                                    flywheelPower = shooterRPMVars.curPower;
                                    sleep(10);
                                }
                                flywheelServo.setPosition(shooterServoRestPos);
                                for (int j = 0; j < 20; j++) {
                                    SetRPM(-150, flywheelPower);
                                    flywheelPower = shooterRPMVars.curPower;
                                    sleep(10);
                                }


                                if (i == 0) {
                                    rotate(6 * degreeSign, 0.4);
                                }


                                if (i == 1) {
                                    rotate(6 * degreeSign, 0.4);
                                }

                            }
                        }


                    }
                    else {

                    }
                }
                 if (shootOverride)   shoot3times(flywheelPower);
            }
            // blockers up/down
            if (gamepad1.y){
                if (blockersDown == false){
                    ringBlockerLeft.setPosition(leftBlockerBlockingPos);
                    ringBlockerRight.setPosition(rightBlockerBlockingPos);
                    blockersDown = true;
                    sleep(100);
                }
                else {
                    ringBlockerLeft.setPosition(leftBlockerRestPos);
                    ringBlockerRight.setPosition(rightBlockerRestPos);
                    blockersDown = false;
                    sleep(100);

                }
            }

            if (gamepad1.dpad_right) rotate(90,0.35);
            // shooter off
            if (gamepad1.dpad_down == true) shoot3times(flywheelPower);
            // shoot 1x

            if (gamepad1.dpad_left == true) {
                flywheelServo.setPosition(shooterServoFlickPos);
                sleep(500);
                flywheelServo.setPosition(shooterServoRestPos);
            }
            // shooter pwr up or down
            if (gamepad2.b) {
                targetRPMGoal -= 1;
                sleep(100);
            }

            if (gamepad2.a) {
                targetRPMGoal += 1;
                sleep(100);
            }



            flywheelPower = calibPwr * calibMult;


            // intake n
            if (gamepad1.x == true) {

                intakeTop.setPower(-1 * intakeTopPwr);
                intakeBottom.setPower(-1 * intakeBottomPwr);

               // frontIntakeFront.setPower(1);
               // frontIntakeBack.setPower(1);

            }


            // intake reverse

            if (gamepad1.b == true) {
                intakeBottom.setPower(intakeBottomPwr);


                intakeTop.setPower(intakeTopPwr);

               // frontIntakeFront.setPower(-1);
               // frontIntakeBack.setPower(-1);


            }


            // calib to goal
            if (gamepad2.x) {
                firstGoalShot = true;
                calibMult = 1.0;
                shooterRPMVars.isPowershot = false;

            }

            // shooter trigger fwd and back
           // if (gamepad1.a) {
             //   flywheelServo.setPosition(shooterServoFlickPos);
              //  sleep(500);
               // flywheelServo.setPosition(shooterServoRestPos);
                //flywheelShooter.setPower(PowershotPower);
           // }
            // power shot calib
            if (gamepad2.y) {

                shooterRPMVars.isPowershot = true;
                shooterRPMVars.isValid = true;

            }
            // take old pos reading
            oldLFPos = LFPos;
            oldLBPos = LBPos;
            oldRFPos = RFPos;
            oldRBPos = RBPos;
            telemetry.addData("Target RPM = ", targetRPMGoal);
            telemetry.addData("color =", pipeline.getColor());
            telemetry.addData("object =", pipeline.noObject);

            //   telemetry.addData("Mult = ", calibMult);
            //telemetry.addData("First Shot = ", firstGoalShot);
            //telemetry.addData("power flip = ",LFflip);



            telemetry.update();


        }


    }

    public double getRPMIntake(double waitTime) {
        ElapsedTime timer = new ElapsedTime();
        double startFWCount = intakeTop.getCurrentPosition();
        while (timer.milliseconds() < waitTime) {
        }

        double timeVar = (250.0 / waitTime);
        double deltaFW = intakeTop.getCurrentPosition() - startFWCount;

        double RPM = timeVar * (deltaFW * 240) / 537.6;

        return RPM;
    }

    public double SetRPMIntake(double targetRPM, double motorPower) {

        double time_step = 100.0;

        double time_step_mul = time_step / 50.0;

        double kp = 0.0025 * 0.1;
        double ki = (0.0025 / 50.0) * 0.1 * 0;
        double kd = 0.0005 * 0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double errorRPM = targetRPM + getRPMIntake(time_step);
        double curPower = motorPower;
        double lastErr = 0;
        double integralErr = 0;
        int inLockCount = 0;
        int loop_count = 0;
        while (loop_count < 50) {
            double deltaError = errorRPM - lastErr;
            lastErr = errorRPM;
            double time_int = timer.time();
            timer.reset();

            double derivative = deltaError / time_int;


            if (Math.abs(errorRPM) < 5) {
                integralErr += errorRPM * time_int;
            } else {
                integralErr += 0;
//                integralErr += ((errorRPM > 0) ? 5 * time_int : -5 * time_int) ;
            }

            double deltaPower = -1 * time_step_mul * ((errorRPM * kp) + (integralErr * ki) + (derivative * kd));

            /* double pwrMul = (Math.abs(errorRPM) > 20) ? 1.0 :
                            (Math.abs(errorRPM) > 10)  ? 1.0/4.0 :
                            (Math.abs(errorRPM) > 5)  ? 1.0/16.0 :
                                    (Math.abs(errorRPM) > 2.5)  ? 01.0/64.0 : (1.0/128.0) ;

             */
            double pwrMul = 1.0;
            curPower += (deltaPower * pwrMul);

            //if (curPower > 0.7) curPower = 0.7 ;
            //if (curPower < -0.7) curPower = -0.7 ;

            intakeTop.setPower(curPower);
            double RPM = getRPMIntake(time_step);
            errorRPM = targetRPM + RPM;
            telemetry.addData("RPMIntake = ", RPM);
            telemetry.addData("errorRPM = ", errorRPM);
            telemetry.addData("curPower  = ", curPower);
            telemetry.addData("deltaPower  = ", deltaPower);
            telemetry.update();

            if (Math.abs(errorRPM) < 5) {
                inLockCount += 1;
                if (inLockCount > 5) {
                    return (curPower);
                }
            } else {
                inLockCount = 0;
            }
        }
        return (curPower);
    }
    public void shoot3times (double flywheelPower){
        /*
        for (int j = 0; j < 10 ; j++){
            SetRPM(targetRPMGoal,flywheelPower);
            sleep(10);
        }
        for (int i = 0; i < 3; i += 1) {
            flywheelServo.setPosition(shooterServoFlickPos);
            if (i == 0) flywheelPower = flywheelPower * 1.1;
            if ( i == 1)  flywheelPower = flywheelPower * 1.125;
            flywheelShooter.setPower(flywheelPower);

            sleep(400);
            flywheelServo.setPosition(shooterServoRestPos);
            sleep(400);


        }

        flywheelShooter.setPower(flywheelPower);

         */

        int loopCount = 15;
        for (int j = 0; j < loopCount ; j++){
            SetRPM(targetRPMGoal,flywheelPower);

        }
        for (int i = 0; i < 3; i += 1) {
            flywheelServo.setPosition(shooterServoFlickPos);
            for (int j = 0; j < loopCount ; j++){
                flywheelPower = SetRPM(targetRPMGoal,flywheelPower);
                //sleep(10);
            }
            flywheelServo.setPosition(shooterServoRestPos);
            for (int j = 0; j < loopCount ; j++){
                flywheelPower = SetRPM(targetRPMGoal,flywheelPower);
              //  sleep(10);
            }

        }


        }


    public double getRPM(double waitTime) {
        ElapsedTime timer = new ElapsedTime();
        double startFWCount = flywheelShooter.getCurrentPosition();
        while (timer.milliseconds() < waitTime) {
        }

        double timeVar = (250.0 / waitTime);
        double deltaFW = flywheelShooter.getCurrentPosition() - startFWCount;

        double RPM = timeVar * (deltaFW * 240) / 537.6;

        return RPM;

    }


    public double SetRPM(double targetRPM, double motorPower) {
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
            shooterRPMVars.inWhile = true ;
            shooterRPMVars.curTime = shooterRPMVars.timer.time() ;
            shooterRPMVars.timer.reset();
        }
        else {
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

            if (shooterRPMVars.curPower > 1.0) shooterRPMVars.curPower = 1.0;
            if (shooterRPMVars.curPower < -1.0) shooterRPMVars.curPower = -1.0;

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

            if (Math.abs(shooterRPMVars.errorRPM) < 1.5) {
                if (shooterRPMVars.inLockCount > 1) {
                    shooterRPMVars.pwrMul = 0.5;
                }
                shooterRPMVars.inLockCount += 1;
                if (shooterRPMVars.inLockCount > 5) {
                    shooterRPMVars.inWhile = false;
                    shooterRPMVars.isValid = false;
                    return (shooterRPMVars.curPower);

                }
            } else {
                shooterRPMVars.inLockCount = 0;
                shooterRPMVars.pwrMul = 1.0;
            }
            //               }
        }
        if (shooterRPMVars.loop_count > 1000){
            //shooterRPMVars.inWhile = false;
            //shooterRPMVars.isValid = false;

        }
        return (shooterRPMVars.curPower);
    }

    public void shooterTrigger3x() {
        double flywheelPower = 0.47;

        for (int i = 0; i < 3; i += 1) {
            double targetRPM = -155;
            flywheelPower = SetRPM(targetRPM, flywheelPower);
            flywheelPower = 1.0 * flywheelPower;
            flywheelServo.setPosition(shooterServoFlickPos);
            sleep(500);
            flywheelServo.setPosition(shooterServoRestPos);

        }
        sleep(500);

        flywheelShooter.setPower(0);

    }

    public double shooterTrigger3xNP(double flywheelPower, double targetRPMGoal, SetRPMVars shooterRPMVars) {
        intakeTop.setPower(0);
        shooterRPMVars.isValid = true;
       // SetRPM(targetRPMGoal, flywheelPower );
        telemetry.addData("Target RPM = ", targetRPMGoal);
        telemetry.update();

        for (int i = 0; i < 3; i += 1) {
            flywheelServo.setPosition(shooterServoFlickPos);
            sleep(500);
            intakeBottom.setPower(intakeBottomShooterPwr);

            flywheelShooter.setPower(flywheelPower * 1.075);
            flywheelServo.setPosition(shooterServoRestPos);
            sleep(500);
            intakeBottom.setPower(0);

        }
        sleep(500);


        flywheelShooter.setPower(0);
        //  intakeTop.setPower(0);


        return (flywheelPower);

    }

    public void shooterTrigger1x(double targetRPM) {


        double flywheelPower = 0.47;
        flywheelPower = SetRPM(targetRPM, flywheelPower);
        flywheelPower = 1.02 * flywheelPower;
        flywheelServo.setPosition(shooterServoFlickPos);
        sleep(500);
        flywheelServo.setPosition(shooterServoRestPos);
        // return (flywheelPower);
        //sleep(0) ;
    }

    public double shooterTrigger1xR(double targetRPM) {
        double flywheelPower = 0.47;
        flywheelPower = SetRPM(targetRPM, flywheelPower);
        flywheelPower = 1.0 * flywheelPower;
        return (flywheelPower);
        //sleep(0) ;
    }

    public void moveWPID(double targetXInches, double targetYInches) {


        frontLeft.setPower(0);
        frontRight.setPower(0);

        backRight.setPower(0);
        backLeft.setPower(0);


        double targetXCount = targetXInches * COUNTS_PER_INCH;
        double targetYCount = targetYInches * COUNTS_PER_INCH;
        // get starting X and Y position from encoders
        // and solving from equation

        double initialYPos = (backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2;
        double initialXPos = (backRight.getCurrentPosition() - backLeft.getCurrentPosition()) / 2;
        // adding Count + initial
        double targetXPos = targetXCount + initialXPos;
        double targetYPos = targetYCount + initialYPos;
        // setting up X and Y for loop change
        double currentXPos = initialXPos;
        double currentYPos = initialYPos;
        double kp = 0.000005;
        double ki = 0.00005;
        double kd = 0.0005;
        double integralX = 0;
        double integralY = 0;
        double finalGain = 5;

        double errorX = targetXPos - currentXPos;
        double errorY = targetYPos - currentYPos;

        boolean movementDoneX = (Math.abs(errorX) < 25);
        boolean movementDoneY = (Math.abs(errorY) < 25);

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
        boolean firstPass = true;


        double curPowerLF = 0;
        double curPowerLB = 0;
        double curPowerRF = 0;
        double curPowerRB = 0;

        ElapsedTime timer = new ElapsedTime();
        // start loop while any error is > some number
        while ((!movementDoneX || !movementDoneY)) {

            double deltaXError = firstPass ? 0 : errorX - lastXError;
            double deltaYError = firstPass ? 0 : errorY - lastYError;

            firstPass = false;

            double curTime = timer.time();

            integralX += errorX * curTime;
            integralY += errorY * curTime;

            double derivativeX = deltaXError / curTime;
            double derivativeY = deltaYError / curTime;


            timer.reset();

            if (movementDoneX) deltaKX = 0;
            if (movementDoneY) deltaKY = 0;

            double deltaXPower = deltaKX * ((errorX * kp) + (integralX * ki) + (derivativeX * kd));
            double deltaYPower = deltaKY * ((errorY * kp) + (integralY * ki) + (derivativeY * kd));

            curPowerX = finalGain * deltaXPower;
            curPowerY = finalGain * deltaYPower;
            double powerLowThreshMul = 0;

            if (((Math.abs(curPowerX)) > minPowerX) || ((Math.abs(curPowerY)) > minPowerY))
                powerLowThreshMul = 1;

            double usePwrX = powerLowThreshMul * curPowerX;
            double usePwrY = powerLowThreshMul * curPowerY;

            if (curPowerX > capPowerX) usePwrX = capPowerX;
            if (curPowerX < (-1 * capPowerX)) usePwrX = -1 * capPowerX;

            if (curPowerY > capPowerY) usePwrY = capPowerY;

            if (curPowerY < (-1 * capPowerY)) usePwrY = -1 * capPowerY;

            double PwrRatioX = (curPowerX != 0) ? Math.abs(usePwrX / curPowerX) : 0;
            double PwrRatioY = (curPowerY != 0) ? Math.abs(usePwrY / curPowerY) : 0;

            if (PwrRatioX != PwrRatioY) {
                if ((PwrRatioX != 0) && (PwrRatioX < PwrRatioY)) {
                    usePwrY = PwrRatioX * usePwrY / PwrRatioY;
                }
                if ((PwrRatioY != 0) && (PwrRatioY < PwrRatioX)) {
                    usePwrX = PwrRatioY * usePwrX / PwrRatioX;
                }

            }

            usePwrX = (!movementDoneX) ? usePwrX : 0;
            usePwrY = (!movementDoneY) ? usePwrY : 0;

            curPowerLF = usePwrY + usePwrX;
            curPowerLB = usePwrY - usePwrX;
            curPowerRF = usePwrY - usePwrX;
            curPowerRB = usePwrY + usePwrX;

            backRight.setPower(curPowerRB);
            backLeft.setPower(curPowerLB);

            frontLeft.setPower(curPowerLF);
            frontRight.setPower(curPowerRF);

            sleep(50);

            double posBL = backLeft.getCurrentPosition();
            double posBR = backRight.getCurrentPosition();

            double posFL = frontLeft.getCurrentPosition();
            double posFR = frontRight.getCurrentPosition();

            currentYPos = (posBL + posBR) / 2;
            currentXPos = (posBR - posBL) / 2;

            errorX = (targetXPos - currentXPos);
            errorY = (targetYPos - currentYPos);


            lastXError = errorX;
            lastYError = errorY;

            movementDoneX = (Math.abs(errorX) < 100) || movementDoneX;
            movementDoneY = (Math.abs(errorY) < 100) || movementDoneY;

            telemetry.addData("ErrX = ", errorX);
            telemetry.addData("ErrY = ", errorY);
            telemetry.addData("Front Left Encoder =", posFL);
            telemetry.addData("Front Right Encoder ", posFR);
            telemetry.addData("Back Left Encoder", posBL);
            telemetry.addData("Back Right Encoder =", posBR);
            telemetry.addData("Power ratio x =", PwrRatioX);
            telemetry.addData("Power ratio y =", PwrRatioY);


            telemetry.update();


        }

        double rampMul = 1.0;

        for (int i = 0; i < 5; i++) {
            rampMul -= 0.2;
            frontLeft.setPower(curPowerLF * rampMul);
            frontRight.setPower(curPowerRF * rampMul);

            backRight.setPower(curPowerRB * rampMul);
            backLeft.setPower(curPowerLB * rampMul);
            sleep(10);
        }


    }

    public void Powershots() {
        shooterTrigger1x(-150);
        moveWPID(-9.5, 0);
        shooterTrigger1x(-150);
        moveWPID(-8.5, 0);
        shooterTrigger1x(-150);
        sleep(500);
        flywheelShooter.setPower(0);


    }

    public void PowershotsFast() {
        double flywheelPower = 0.47;
        double targetRPM = -150;
        flywheelPower = SetRPM(targetRPM, flywheelPower);
        for (int i = 0; i < 3; i += 1) {
            flywheelServo.setPosition(shooterServoFlickPos);
            sleep(500);
            flywheelShooter.setPower(flywheelPower * 1.075);
            flywheelServo.setPosition(shooterServoRestPos);
            sleep(500);
            moveWPID(-9, 0);


        }


    }

    public void ArmEncoders(double speed, double distance, int timeoutInMilliseconds) {
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

    public void CommonMethodForArm() {

        ArmEncoders(0.7, -0.88, 10000);
        sleep(500);
        armServo.setPosition(0);
        moveWPID(8, 0);
        ArmEncoders(0.7, 1.3, 10000);
        armServo.setPosition(1);


    }

    public void rotate (double degrees, double power){
        double initYLeft = intakeBottom.getCurrentPosition();
        double initYRight = intakeTop.getCurrentPosition();
        double targetRotationCt = degrees * COUNTS_PER_DEGREE_ODO;
        boolean done = false;
        double powerMult = 1;
        double kp = 1;
        double deltaRot =0;
        //degrees += 5;

        while (!done) {


            double curYLeft = intakeBottom.getCurrentPosition() - initYLeft;
            double curYRight = intakeTop.getCurrentPosition() - initYRight;

            double curRot = (curYRight - curYLeft);
            deltaRot = (Math.abs(targetRotationCt - curRot))/COUNTS_PER_DEGREE_ODO;

            if (deltaRot > 5) kp = 1;
            else kp = deltaRot/5;

            if (degrees > 0) {
                done = (curRot >= targetRotationCt) || (deltaRot < 2);
            }
            else {
                done = (curRot <= targetRotationCt) || (deltaRot < 2);
                powerMult = -1.0;
            }
            telemetry.addData("target rotation Ct", targetRotationCt);
            telemetry.addData("cur rotation", curRot);
            telemetry.addData("delta rot = ", deltaRot);
            telemetry.update();


            if (!done){
                frontRight.setPower(-1 * power * powerMult * kp);
                backRight.setPower(-1 * power * powerMult * kp);
                frontLeft.setPower(power * powerMult * kp);
                backLeft.setPower(power * powerMult * kp);
            }

            telemetry.addData("target rotation Ct", targetRotationCt);
            telemetry.addData("cur rotation", curRot);
            telemetry.addData("delta rot = ", deltaRot);
            telemetry.update();

        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);





    }




}

