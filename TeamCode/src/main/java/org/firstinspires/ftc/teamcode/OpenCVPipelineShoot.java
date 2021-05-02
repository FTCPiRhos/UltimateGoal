package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.vision.OpenCVCrCb;
import org.opencv.core.Core;
import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class OpenCVPipelineShoot extends OpenCvPipeline
{
// 320 x 240
    public OpenCVPipelineShoot() {

            REGION1_TOPLEFT_ANCHOR_POINT = new Point( 0, 0 );
    }
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
    static final Scalar WHITE = new Scalar(255,255,255);
    static final Scalar YELLOW = new Scalar(0,255,255);

    /*
     * The core values which define the location and size of the sample regions
     */
    boolean fLeftPos = false;
    Point REGION1_TOPLEFT_ANCHOR_POINT = null;
    static final int REGION_WIDTH = 320;
    static final int REGION_HEIGHT = 60;
    // this is thresholds for return values
    final double FOUR_RING_THRESHOLD = 0.7;
    final double ONE_RING_THRESHOLD = 0.25;

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
    Point region1_pointA;
    Point region1_pointB;

    Point R1TopLeft;
    Point R2TopLeft;
    Point R1BottomRight;
    Point R2BottomRight;


    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat RGB = new Mat();
    Mat Cb = new Mat();
    int avg0;
    int avg1;
    int avg2;
    double ratio;
    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile OpenCVPipelineShoot.RingPosition position = OpenCVPipelineShoot.RingPosition.FOUR;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        //RGBA -> RGB
       Imgproc.cvtColor(input, RGB,1);
       Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_RGB2YCrCb);
        //Core.extractChannel(YCrCb, Cb, 1);
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

        region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        region1_Cb = YCrCb.submat(new Rect(region1_pointA, region1_pointB));
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
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                -1);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        boolean adjacentRed = false;
        ArrayList<Point> redPoints = new ArrayList<Point>();


        for (int dx = 0; dx < REGION_WIDTH; dx ++){
            for (int dy = 0; dy < REGION_HEIGHT; dy++){
                double [] RGB = input.get(dy,dx);
                region1_pointA = new Point(
                        dx,
                        dy);
                region1_pointB = new Point(
                        dx + 1,
                        dy + 1);

                if (RGB != null) {
                    double red = RGB[0];
                    double green = RGB[1];
                    double blue = RGB[2];

                    //double red = ((298.082 * Y) / 256) + ((408.583 * Cr) / 256) - 222.921;
                    //double green = ((298.082 * Y) / 256) - ((100.291 * Cb) / 256) - ((208.120 * Cr)/256) + 135.576;
                    //double blue = ((298.082 * Y) / 256) + ((516.412 * Cb)/256) - 276.836;



                    if ((red > 30) && (red > (1.5 * green)) && (red > (1.5 * blue)) ){
                        redPoints.add(region1_pointA);
                        /*
                        Imgproc.rectangle(
                                input,
                                region1_pointA,
                                region1_pointB,
                                WHITE);

                         */





                    }

                    }




            }
        }

        double xObject1 = 320;
        double xObject2 = 0;
        for(Point red: redPoints){
            Point rPlus1 = new Point(red.x + 1, red.y + 1);
            if (red.x > xObject2) xObject2 = red.x;
            if (red.x < xObject1) xObject1 = red.x;
            Imgproc.rectangle(
                    input,
                    red,
                    rPlus1,
                    WHITE);
        }


        double R1P1X = 320;
        double R1P1Y = 240;

        double R1P2X = 0;
        double R1P2Y = 0;

        double R2P1X = 320;
        double R2P1Y = 240;

        double R2P2X = 0;
        double R2P2Y = 0;

        for(Point red: redPoints){
            if (Math.abs(red.x - xObject1) < 15){
                // This is Rect 1

                if (red.x < R1P1X) R1P1X = red.x;
                if (red.y < R1P1Y) R1P1Y = red.y;

                if (red.x > R1P2X) R1P2X = red.x;
                if (red.y > R1P2Y) R1P2Y = red.y;


            }

            if (Math.abs(red.x - xObject2) < 15){
                // This is Rect 1

                if (red.x < R2P1X) R2P1X = red.x;
                if (red.y < R2P1Y) R2P1Y = red.y;

                if (red.x > R2P2X) R2P2X = red.x;
                if (red.y > R2P2Y) R2P2Y = red.y;


            }
        }

        R1TopLeft = new Point(R1P1X,R1P1Y);
        R1BottomRight = new Point(R1P2X,R1P2Y);
        R2TopLeft = new Point(R2P1X,R2P1Y);
        R2BottomRight = new Point(R2P2X,R2P2Y);

        Imgproc.rectangle(
                input,
                R1TopLeft,
                R1BottomRight,
                YELLOW,
                -1

        );

        Imgproc.rectangle(
                input,
                R2TopLeft,
                R2BottomRight,
                BLUE,
                -1

        );










        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */

    public RingPosition getAnalysis()
    {
        return position;
    }

    public Point getR1TopLeft(){
        return R1TopLeft;
    }

    public Point getR1BottomRight(){
        return R1BottomRight;
    }

    public Point getR2TopLeft(){
        return R2TopLeft;
    }

    public Point getR2BottomRight(){
        return R2BottomRight;
    }


}

