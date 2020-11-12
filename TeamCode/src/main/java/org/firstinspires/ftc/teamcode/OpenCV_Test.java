package org.firstinspires.ftc.teamcode;

/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple stones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@Autonomous(name = "Blue Tower Goal Align", group = "PiRhos")
public class OpenCV_Test extends UltimateGoalAutonomousBaseOpenCV
{
    OpenCvInternalCamera2 phoneCam;
    StoneOrientationAnalysisPipeline pipeline;

    private static final int TARGET_OBJECT_TO_FIND = 2;
    private static final double LEFT_OBJECT_X_POS_LOWER = 25;
    private static final double LEFT_OBJECT_X_POS_HIGHER = 35;
    private static final double RIGHT_OBJECT_X_POS_LOWER = 100;
    private static final double RIGHT_OBJECT_X_POS_HIGHER = 110;


    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new StoneOrientationAnalysisPipeline();
                phoneCam.setPipeline(pipeline);
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive())
        {

            // Don't burn an insane amount of CPU cycles in this sample because
            // we're not doing anything else
            sleep(100);

            initHardware();

            // Figure out which stones the pipeline detected, and print them to telemetry
            ArrayList<StoneOrientationAnalysisPipeline.AnalyzedStone> stones = pipeline.getDetectedStones();
            if(stones.isEmpty())
            {
                telemetry.addLine("No objects detected");
            }
            else
            {
                int objectFound = 0;
                double object_x_pos[] = new double[10];
                for(StoneOrientationAnalysisPipeline.AnalyzedStone stone : stones)
                {
                    if (stone.points[1].y < 100) {
                        telemetry.addLine(String.format("Object: Angle=%.2f (%.2f,%.2f) (%.2f,%.2f)", stone.angle,
//                                                                                                    stone.points[0].x, stone.points[0].y,
                                stone.points[1].x, stone.points[1].y,
//                                                                                                    stone.points[2].x, stone.points[2].y,
                                stone.points[3].x, stone.points[3].y));
                        if ( objectFound < TARGET_OBJECT_TO_FIND )
                            object_x_pos[objectFound] = stone.points[1].x;
                        objectFound++;
                    }
                }

                telemetry.addData("Object Found : ", objectFound);
                telemetry.update();

                double left_lower_value = 10;
                double right_lower_value = RIGHT_OBJECT_X_POS_LOWER - 40;
                double left_higher_value = LEFT_OBJECT_X_POS_HIGHER + 40;
                double right_higher_value = RIGHT_OBJECT_X_POS_HIGHER + 40;

                if ( objectFound == TARGET_OBJECT_TO_FIND ){
                    if (object_x_pos[0] < object_x_pos[1]){
                        if ( object_x_pos[0] < LEFT_OBJECT_X_POS_LOWER && object_x_pos[1] < RIGHT_OBJECT_X_POS_LOWER ) {
                            if ( object_x_pos[0] < left_lower_value && object_x_pos[1] < right_lower_value )
                                moveSidewayForMilliseconds(0.4, 1000);
                            else
                                moveSidewayForMilliseconds(0.2, 600);
                            telemetry.addLine("left move");
                            }
                        else if ( object_x_pos[0] > LEFT_OBJECT_X_POS_HIGHER && object_x_pos[1] > RIGHT_OBJECT_X_POS_HIGHER ) {
                            if ( object_x_pos[0] > left_higher_value && object_x_pos[1] > right_higher_value )
                                moveSidewayForMilliseconds(-0.4, 1000);
                            else
                                moveSidewayForMilliseconds(-0.2, 600);
                            telemetry.addLine("right move");
                        }
                        else {
                            stop();
                            telemetry.addLine("stopped");
                        }
                    }
                    else {
                        if ( object_x_pos[0] < RIGHT_OBJECT_X_POS_LOWER && object_x_pos[1] < LEFT_OBJECT_X_POS_LOWER ) {
                            if ( object_x_pos[0] < right_lower_value && object_x_pos[1] < left_lower_value )
                                moveSidewayForMilliseconds(0.4, 1000);
                            else
                                moveSidewayForMilliseconds(0.2, 600);
                            telemetry.addLine("LEFT MOVE");
                        }
                        else if ( object_x_pos[0] > RIGHT_OBJECT_X_POS_HIGHER && object_x_pos[1] > LEFT_OBJECT_X_POS_HIGHER ) {
                            if ( object_x_pos[0] > right_higher_value && object_x_pos[1] > left_higher_value )
                                moveSidewayForMilliseconds(-0.4, 1000);
                            else
                                moveSidewayForMilliseconds(-0.2, 600);
                            telemetry.addLine("RIGHT MOVE");
                        }
                        else {
                            stop();
                            telemetry.addLine("STOPPED");
                        }
                    }
                }
            }

        }
    }
}
