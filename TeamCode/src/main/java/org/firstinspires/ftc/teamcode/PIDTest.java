package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="PID Test", group="PiRhos")


@Disabled public class PIDTest extends UltimateGoalAutonomousBase {
    @Override
    public void runOpMode() {

    }

    public double getRPM(){
        double waitTime = 250;
        ElapsedTime timer = new ElapsedTime ();
        double startFWCount = FW.getCurrentPosition();
        while (timer.milliseconds() < waitTime)
        {
        }

        double deltaFW = FW.getCurrentPosition() - startFWCount;

        double RPM = (deltaFW * 240)/537.6;

        return RPM;

    }

    public void FWPID () {
        double kp = 0.1;
        double ki = 0.1;
        double kd = 0.1;
        double targetRPM = 6000;
        double error = getRPM() - targetRPM;
        double lastError = 0;
        double integral = 0;
        ElapsedTime timer = new ElapsedTime();
        while (Math.abs(error) >=10){


            error = getRPM() - targetRPM;
            double deltaError = lastError - error;
            integral += deltaError * timer.time();
            double derivative = deltaError / timer.time();
            FW.setPower(kp * error + ki * integral + kd * derivative);
            error = lastError;
            timer.reset();
        }
    }

        public void getEncoder(){
        double firstPos = FW.getCurrentPosition();
        FW.setPower(1);
        ElapsedTime timer = new ElapsedTime();
        while(timer.milliseconds()<1000)
        {
            sleep(100);
        }
        double newPos = FW.getCurrentPosition();

        double deltaPos = newPos - firstPos;

            telemetry.addData("Status", "Encoder Change: " + deltaPos);
            telemetry.update();
        }
    public void MoveFwdPID (double targetInches) {
        double kp = 0.1;
        double ki = 0.1;
        double kd = 0.1;
        double targetDistance = targetInches * COUNTS_PER_INCH;
        double errorFL = frontLeft.getCurrentPosition() - targetDistance;
        double errorFR = frontRight.getCurrentPosition() - targetDistance;
        double errorBL = backLeft.getCurrentPosition() - targetDistance;
        double errorBR = backRight.getCurrentPosition() - targetDistance;
        double lastErrorFL = 0;
        double lastErrorFR = 0;
        double lastErrorBL = 0;
        double lastErrorBR = 0;
        double integral = 0;
        ElapsedTime timer = new ElapsedTime();
        while (Math.abs(errorFL) >=200 ||Math.abs(errorFR) >=200 || Math.abs(errorBL) >=200  ) {

            errorFL = frontLeft.getCurrentPosition() - targetDistance;
            double deltaError = lastErrorFL - errorFL;
            integral += deltaError * timer.time();
            double derivative = deltaError / timer.time();
            frontLeft.setPower(kp * errorFL + ki * integral + kd * derivative);
            errorFL = lastErrorFL;
            timer.reset();
        }
    }

}
