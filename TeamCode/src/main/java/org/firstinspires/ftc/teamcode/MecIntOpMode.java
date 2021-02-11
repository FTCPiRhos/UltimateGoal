package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Intake Op Mode FR", group="PiRhos")

@Disabled public class MecIntOpMode extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor armMotor = null;
    private Servo armServo = null;
    private Servo flywheelServo = null;
    private DcMotor flywheelShooter = null;
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0 / 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.937;   // For figuring circumference - 100mm
    static final double COUNTS_PER_INCH = 1.45 * (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED_SLOW = 0.4;
    static final double DRIVE_SPEED = 0.7;

    // servo is at port 0 of main
    // motor is at port 3 of secondary


    @Override
    public void runOpMode(){
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
        armServo = hardwareMap.get(Servo.class,"arm_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection((DcMotor.Direction.REVERSE));
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);


        intake1 = hardwareMap.get(DcMotor.class,"intake1");
        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        flywheelShooter = hardwareMap.get(DcMotor.class, "flywheel_shooter");
        flywheelShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flywheelShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelServo = hardwareMap.get(Servo.class, "flywheel_servo");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            // init variables
            double LFPower;
            double LBPower;
            double RFPower;
            double RBPower;
            double ArmPower;

            double arm = gamepad2.right_stick_y ;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            // calculate motor powers
            LFPower = (y + x + rx);
            LBPower = (y - x + rx);
            RFPower = (y - x - rx);
            RBPower = (y + x - rx);
            ArmPower = arm;
            // make sure none of the drive powers are too high
            if (Math.abs(LFPower) > 1 || Math.abs(LBPower) > 1 ||
                    Math.abs(RFPower) > 1 || Math.abs(RBPower) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(LFPower), Math.abs(LBPower));
                max = Math.max(Math.abs(max), Math.abs(RFPower));
                max = Math.max(Math.abs(max),Math.abs(RBPower) );

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                LFPower /= max;
                LBPower /= max;
                RFPower /= max;
                RBPower /= max;
            }

            LFPower = LFPower * 0.75;
            LBPower = LBPower * 0.75;
            RFPower = RFPower * 0.75;
            RBPower = RBPower * 0.75;

            frontLeft.setPower(LFPower);
            backLeft.setPower(LBPower);
            frontRight.setPower(RFPower);
            backRight.setPower(RBPower);
            armMotor.setPower(ArmPower);

            if (gamepad1.left_bumper == true) armServo.setPosition(0);



            if (gamepad1.right_bumper == true) armServo.setPosition(1);





            //double intake1pwr = gamepad2.left_stick_y;
            //double intake2pwr = gamepad2.left_stick_y;
            if (gamepad1.b == true){
            double I1Pwr = 0.6;
            double I2Pwr = -1 ;

            intake1.setPower(I1Pwr);
            intake2.setPower(I2Pwr);

        }
            if (gamepad1.y == true){
                double I1Pwr = 0;
                double I2Pwr = 0 ;

                intake1.setPower(I1Pwr);
                intake2.setPower(I2Pwr);
            }
            if (gamepad1.x == true){
                double I1Pwr = -0.6;
                double I2Pwr = 1 ;

                intake1.setPower(I1Pwr);
                intake2.setPower(I2Pwr);

            }

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

}
