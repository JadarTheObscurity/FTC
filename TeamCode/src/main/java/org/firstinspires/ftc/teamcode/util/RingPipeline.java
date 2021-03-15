package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingPipeline extends OpenCvPipeline {

    Point resolution = new Point(640, 480);


    public RingPipeline(Point resolution){
        this.resolution = resolution;
    }
    public RingPipeline(){}

    Mat thresh = new Mat();
    Mat thresh_region = new Mat();
    Mat ROI = new Mat();
    HSV_threshold threshold = new HSV_threshold(0, 255, 0, 255, 0, 255);
    public int min_y = 0;
    public int max_y = 255;
    public int min_cr = 0;
    public int max_cr = 117;
    public int min_cb = 140;
    public int max_cb = 255;

    public RingStatus ringStatus = RingStatus.NONE;
    int threshold_1 = 250;
    int threshold_2 = 900;

    public int white_pixel = 0;

    Mat YCrCb = new Mat();
    Mat region;
    Point pointA = new Point(5 * resolution.x / 8, 1 * resolution.y/8);
    Point pointB = new Point(7 * resolution.x / 8, 3 * resolution.y/8);

    public enum RingStatus {
        NONE,
        ONE,
        FOUR
    }

    enum Stage{
        Origin,
        Thresh,
        ROI
    }
    public static int curr_stage = 0;


    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    private void toYCrCb(Mat input){
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);
    }


    @Override
    public void init(Mat input){
        toYCrCb(input);

        region = YCrCb.submat(new Rect(pointA, pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        Imgproc.resize(input, input, new Size(resolution.x, resolution.y));
        toYCrCb(input);
        Core.inRange(YCrCb, new Scalar(min_y, min_cr, min_cb), new Scalar(max_y, max_cr, max_cb), thresh);
        Core.inRange(region, new Scalar(min_y, min_cr, min_cb), new Scalar(max_y, max_cr, max_cb), thresh_region);

        white_pixel = Core.countNonZero(thresh_region);

        Imgproc.rectangle(
                thresh,
                pointA,
                pointB,
                new Scalar(255, 255, 255), 4);
        Imgproc.rectangle(
                input,
                pointA,
                pointB,
                new Scalar(0, 255, 0), 4);


        if(white_pixel <= threshold_1) ringStatus = RingStatus.NONE;
        else if(white_pixel <= threshold_2) ringStatus = RingStatus.ONE;
        else ringStatus = RingStatus.FOUR;

        if(curr_stage >= Stage.values().length) curr_stage = 0;
        if(curr_stage == Stage.Origin.ordinal()) return input;
        if(curr_stage == Stage.Thresh.ordinal()) return  thresh;
        if(curr_stage == Stage.ROI.ordinal()) {
            ROI = new Mat(thresh, new Rect(pointA, pointB));
            Imgproc.resize(ROI, ROI, new Size(resolution.x, resolution.y));
            return ROI;
        }
        return thresh;
    }

    public void setYCrCbThreshold(int min_y,
                                  int max_y,
                                  int min_cr,
                                  int max_cr,
                                  int min_cb,
                                  int max_cb){

        this.min_y = min_y;
        this.max_y = max_y;
        this.min_cr = min_cr;
        this.max_cr = max_cr;
        this.min_cb = min_cb;
        this.max_cb = max_cb;
    }

}
