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
public class OpenCVPipelineShoot extends OpenCvPipeline {
// 320 x 240




    public OpenCVPipelineShoot(boolean red) {
        Red = red;
        if(Red) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 0);
            REGION1_TOPLEFT_ANCHOR_POINT1 = new Point(255, 175);
            REGION1_TOPLEFT_ANCHOR_POINT2 = new Point(5, 175);
        }
        else {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 0);
            REGION1_TOPLEFT_ANCHOR_POINT1 = new Point(260, 125);
            REGION1_TOPLEFT_ANCHOR_POINT2 = new Point(0, 125);
        }

    }

    /*
     * An enum to define the skystone position
     */
    public enum RingPosition {
        FOUR,
        ONE,
        NONE,
        UNKNOWN
    }

    public enum StartRobotPosition {
        LEFT,
        RIGHT,
        UNKNOWN
    }

    public enum Color {
        RED,
        BLUE,
        UNKNOWN
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    boolean noObject = false;

    /*
     * The core values which define the location and size of the sample regions
     */
    boolean Red = false;
    Point REGION1_TOPLEFT_ANCHOR_POINT1 = null;
    Point REGION1_TOPLEFT_ANCHOR_POINT2 = null;
    Point REGION1_TOPLEFT_ANCHOR_POINT = null;

    static final int REGION_WIDTH1 = 59;
    static final int REGION_HEIGHT1 = 89;
    // this is thresholds for return values
    final double FOUR_RING_THRESHOLD = 45;
    final double ONE_RING_THRESHOLD = 15;

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
    Point region1_pointA1;
    Point region1_pointB1;

    Point region1_pointA2;
    Point region1_pointB2;


    static final Scalar WHITE = new Scalar(255, 255, 255);
    static final Scalar YELLOW = new Scalar(0, 255, 255);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final int REGION_WIDTH = 320;
    static final int REGION_HEIGHT = 40;
    // this is thresholds for return values

    /*
     * Working variables
     */
    Mat region1_Cb1;
    Mat YCrCb1 = new Mat();
    Mat region1_Cb2;
    Mat YCrCb2 = new Mat();
    Mat Cb1 = new Mat();
    int avg0;
    int avg1;
    int avg2;
    int numRows;
    int numColumns;
    double ratio;
    double orangeCt = 0;
    double totalRed = 0;
    double totalBlue = 0;
    double totalGreen = 0;

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

    Mat Cb2 = new Mat();


    // Volatile since accessed by OpMode thread w/o synchronization
    // position is ring size
    private volatile org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.RingPosition position = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.RingPosition.UNKNOWN;
    // initrobotpos is left or right
    private volatile org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.StartRobotPosition initRobotPos = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.StartRobotPosition.UNKNOWN;
    // color is alliance color
    private volatile org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.Color color = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.Color.UNKNOWN;



    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
       Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
       Core.extractChannel(YCrCb, Cb, 1);
        Imgproc.cvtColor(input, YCrCb1, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb1, Cb1, 1);
        Imgproc.cvtColor(input, YCrCb2, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(YCrCb2, Cb2, 1);

    }

    @Override
    public void init(Mat firstFrame) {
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




        region1_pointA1 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT1.x,
                REGION1_TOPLEFT_ANCHOR_POINT1.y);
        region1_pointB1 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT1.x + REGION_WIDTH1,
                REGION1_TOPLEFT_ANCHOR_POINT1.y + REGION_HEIGHT1);
        region1_Cb1 = YCrCb1.submat(new Rect(region1_pointA1, region1_pointB1));



        region1_pointA2 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT2.x,
                REGION1_TOPLEFT_ANCHOR_POINT2.y);
        region1_pointB2 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT2.x + REGION_WIDTH1,
                REGION1_TOPLEFT_ANCHOR_POINT2.y + REGION_HEIGHT1);
        region1_Cb2 = YCrCb2.submat(new Rect(region1_pointA2, region1_pointB2));




    }

    @Override
    public Mat processFrame(Mat input) {

        boolean detectedRed = getColor() == Color.RED;
        boolean detectedBlue = getColor() == Color.BLUE;
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
        avg0 = (int) Core.mean(region1_Cb1).val[0];
        avg1 = (int) Core.mean(region1_Cb1).val[1];
        avg2 = (int) Core.mean(region1_Cb1).val[2];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA1, // First point which defines the rectangle
                region1_pointB1, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        //position = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.RingPosition.UNKNOWN;// Record our analysis
        //initRobotPos = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.StartRobotPosition.UNKNOWN;


        // (cr - cb) / y
        ratio = ((double) (avg1 - avg2)) / ((double) avg0);

        ArrayList<Point> orangePointsLeft = new ArrayList<Point>();
        ArrayList<Point> orangePointsRight = new ArrayList<Point>();

        orangeCt = 0;
        totalBlue = 0;
        totalRed = 0;
        totalGreen = 0;
        for (int dx = 0; dx < REGION_WIDTH1; dx++) {
            for (int dy = 0; dy < REGION_HEIGHT1; dy++) {
                double[] RGB = input.get((int)(REGION1_TOPLEFT_ANCHOR_POINT1.y) + dy, (int)(REGION1_TOPLEFT_ANCHOR_POINT1.x) + dx);
                if (RGB != null) {
                    double red = RGB[0];
                    double green = RGB[1];
                    double blue = RGB[2];

                    boolean isOrange = ((red > 10) && (red >1.2 * green) && (red+green > 1.5 * blue)) && (green > 20) && (blue < 30);

                    if (isOrange) {

                        orangeCt++;
                        totalRed += red;
                        totalGreen += green;
                        totalBlue += blue;
                        region1_pointA = new Point(
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT1.x) + dx,
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT1.y) + dy);
                        region1_pointB = new Point(
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT1.x) + dx + 1,
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT1.y) + dy + 1);

                        orangePointsLeft.add(region1_pointA);

                        Imgproc.rectangle(
                                input,
                                region1_pointA,
                                region1_pointB,
                                BLUE);
                    }
                }
            }
        }

        double leftStackHeight = 0;
        double rightStackHeight = 0;
        double realStackHeight = 0;
        double minYLeft = 240;
        double maxYLeft = 0;
        double minYRight = 240;
        double maxYRight = 0;
        for (int dx = 0; dx < REGION_WIDTH1; dx++) {
            for (int dy = 0; dy < REGION_HEIGHT1; dy++) {
                double[] RGB = input.get((int)(REGION1_TOPLEFT_ANCHOR_POINT2.y) + dy, (int)(REGION1_TOPLEFT_ANCHOR_POINT2.x) + dx);
                if (RGB != null) {
                    double red = RGB[0];
                    double green = RGB[1];
                    double blue = RGB[2];

                    boolean isOrange = ((red > 10) && (red >1.2 * green) && (red+green > 1.5 * blue));

                    if (isOrange) {

                        region1_pointA = new Point(
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT2.x) + dx,
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT2.y) + dy);
                        region1_pointB = new Point(
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT2.x) + dx + 1,
                                (int)(REGION1_TOPLEFT_ANCHOR_POINT2.y) + dy + 1);

                        orangePointsRight.add(region1_pointA);

                        Imgproc.rectangle(
                                input,
                                region1_pointA,
                                region1_pointB,
                                BLUE);
                    }
                }
            }
        }

        if((orangePointsLeft.size() > 20) || (orangePointsRight.size() > 20)) {
            for (Point orange : orangePointsLeft) {

                if (orange.y > maxYLeft) maxYLeft = orange.y;
                if (orange.y < minYLeft) minYLeft = orange.y;


            }

            for (Point orange : orangePointsRight) {

                if (orange.y > maxYRight) maxYRight = orange.y;
                if (orange.y < minYRight) minYRight = orange.y;


            }

            leftStackHeight = maxYLeft - minYLeft;
            rightStackHeight = maxYRight - minYRight;

            if(leftStackHeight > rightStackHeight) {
                realStackHeight = leftStackHeight;
                initRobotPos = StartRobotPosition.LEFT;
            }
            else{
                realStackHeight = rightStackHeight;
                initRobotPos = StartRobotPosition.RIGHT;
            }
        }


        // ratio > .7
        if (realStackHeight > FOUR_RING_THRESHOLD) {
            position = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.RingPosition.FOUR;
        }
        // ratio > .25
        else if (realStackHeight > ONE_RING_THRESHOLD) {
            position = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.RingPosition.ONE;
        } else {
             position = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.RingPosition.NONE;
        }

        /*
         * Draw a solid rectangle on top of the chosen region.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA1, // First point which defines the rectangle
                region1_pointB1, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                0); // Negative thickness means solid fill

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA2, // First point which defines the rectangle
                region1_pointB2, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                0);

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */


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

            ArrayList<Point> redPoints = new ArrayList<Point>();
        double redCt = 0;
        double blueCt = 0;

            for (int dx = 0; dx < REGION_WIDTH; dx++) {
                for (int dy = 0; dy < REGION_HEIGHT; dy++) {
                    double[] RGB = input.get(dy, dx);
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
                        double redThresh = 12;
                        double blueThresh = 15;

                        boolean isRed = ((red > redThresh) && (red > (1.75 * green)) && (red > (1.75 * blue)) && (!detectedBlue) && (blue < redThresh) && (green < redThresh));
                        if (isRed) redCt++;

                        boolean isBlue = (((blue > blueThresh) && (blue > (2.0 * green)) && (blue > (2.0 * red))) && (dx <300) && (!detectedRed) && (red < blueThresh) && (green < blueThresh));
                        if (isBlue) blueCt++;

                        if (isRed || isBlue) {
                            redPoints.add(region1_pointA);

                        Imgproc.rectangle(
                                input,
                                region1_pointA,
                                region1_pointB,
                                WHITE);
                        }
                    }
                }
            }

            double xObject1 = 320;
            double xObject2 = 0;

            boolean oneObject = false;

                for (Point red : redPoints) {
                    Point rPlus1 = new Point(red.x + 1, red.y + 1);
                    if (red.x > xObject2) xObject2 = red.x;
                    if (red.x < xObject1) xObject1 = red.x;
                    Imgproc.rectangle(
                            input,
                            red,
                            rPlus1,
                            BLUE);
                }

                oneObject = Math.abs(xObject1-xObject2) < 30;

                noObject = ((xObject1 == 320) && (xObject2 == 0) )  || (color == org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.Color.UNKNOWN);




                double R1P1X = 320;
                double R1P1Y = 240;

                double R1P2X = 0;
                double R1P2Y = 0;

                double R2P1X = 320;
                double R2P1Y = 240;

                double R2P2X = 0;
                double R2P2Y = 0;

                for (Point red : redPoints) {
                    if (Math.abs(red.x - xObject1) < 15) {
                        // This is Rect 1

                        if (red.x < R1P1X) R1P1X = red.x;
                        if (red.y < R1P1Y) R1P1Y = red.y;

                        if (red.x > R1P2X) R1P2X = red.x;
                        if (red.y > R1P2Y) R1P2Y = red.y;


                    }

                    if (Math.abs(red.x - xObject2) < 15) {
                        // This is Rect 1

                        if (red.x < R2P1X) R2P1X = red.x;
                        if (red.y < R2P1Y) R2P1Y = red.y;

                        if (red.x > R2P2X) R2P2X = red.x;
                        if (red.y > R2P2Y) R2P2Y = red.y;


                    }
                }

                R1TopLeft = new Point(R1P1X, R1P1Y);
                R1BottomRight = new Point(R1P2X, R1P2Y);
                R2TopLeft = new Point(R2P1X, R2P1Y);
                R2BottomRight = new Point(R2P2X, R2P2Y);

                Imgproc.rectangle(
                        input,
                        R1TopLeft,
                        R1BottomRight,
                        BLUE,
                        -1

                );

                Imgproc.rectangle(
                        input,
                        R2TopLeft,
                        R2BottomRight,
                        BLUE,
                        -1

                );

                if ((redCt > blueCt) && (redCt > 20)) {
                    color = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.Color.RED;
                }
                else if (blueCt > 20){
                    color = org.firstinspires.ftc.teamcode.OpenCVPipelineShoot.Color.BLUE;
                }
                else{
                    color = Color.UNKNOWN;
                }



        return input;
    }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public RingPosition getAnalysis ()
        {
            return position;
        }

        public void setAnalysis(RingPosition pos){
            position = pos;
        }

       public StartRobotPosition getInitRobotPos(){
            return initRobotPos;
       }

       public void setInitRobotPos(StartRobotPosition pos){
        initRobotPos = pos;
    }

       public Color getColor(){
            return color;
       }

       public void setColor(Color col){
            color = col;
       }

        public int NumRows () {
            return numRows;
        }

        public int NumColumns () {
            return numColumns;
        }

        public double getOrangeCt(){
            return  orangeCt;
        }

        public double getTotalRed(){
            return totalRed;
        }

    public double getTotalBlue(){
        return totalBlue;
    }

        public double getTotalGreen(){
            return totalGreen;
        }
        /*
         * An enum to define the skystone position
         */


        /*
         * Some color constants
         */




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


        // Volatile since accessed by OpMode thread w/o synchronization

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */






        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */


        public Point getR1TopLeft () {
            return R1TopLeft;
        }

        public Point getR1BottomRight () {
            return R1BottomRight;
        }

        public Point getR2TopLeft () {
            return R2TopLeft;
        }

        public Point getR2BottomRight () {
            return R2BottomRight;
        }


    }



