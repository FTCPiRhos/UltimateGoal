package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Intake Op Mode", group="PiRhos")

public class MecOpModeInt extends LinearOpMode {
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
        /*
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

         */
        intake1 = hardwareMap.get(DcMotor.class,"intake1");
        intake2 = hardwareMap.get(DcMotor.class,"intake2");
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
/*
            // init variables
            double LFPower;
            double LBPower;
            double RFPower;
            double RBPower;
            double ArmPower;

            double arm = gamepad1.right_stick_y ;
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




 */
            double intake1pwr = gamepad1.left_stick_y;
            double intake2pwr = gamepad1.left_stick_y;

            double I1Pwr = intake1pwr * 0.6;
            double I2Pwr = -1 * intake2pwr ;

            intake1.setPower(I1Pwr);
            intake2.setPower(I2Pwr);

        }



    }

}
