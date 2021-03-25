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

    Point show_resolution = new Point(640, 480);
    Point proc_resolution = new Point(160, 120);

    public RingPipeline(Point resolution){
        this.show_resolution = resolution;
    }
    public RingPipeline(){}

    Mat thresh = new Mat();
    Mat thresh_region = new Mat();
    Mat ROI = new Mat();
    Mat proc_Mat = new Mat();
    public static HSV_threshold threshold = new HSV_threshold(10, 30, 0, 255, 100, 255);

    public RingStatus ringStatus = RingStatus.NONE;
    int threshold_1 = 800;
    int threshold_2 = 2500;

    public int white_pixel = 0;

    Point pointA = new Point(1 * show_resolution.x / 8, 5 * show_resolution.y/8);
    Point pointB = new Point(7 * show_resolution.x / 8, 7 * show_resolution.y/8);

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

    @Override
    public void init(Mat input){

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
        Imgproc.resize(input, input, new Size(show_resolution.x, show_resolution.y));
//        Core.inRange(YCrCb, new Scalar(min_y, min_cr, min_cb), new Scalar(max_y, max_cr, max_cb), thresh);
//        Core.inRange(region, new Scalar(min_y, min_cr, min_cb), new Scalar(max_y, max_cr, max_cb), thresh_region);

//        white_pixel = Core.countNonZero(thresh_region);
        determin_ring(input);
//        Imgproc.rectangle(
//                proc_Mat,
//                pointA,
//                pointB,
//                new Scalar(255, 255, 255), 4);
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
        if(curr_stage == Stage.Thresh.ordinal()) return  proc_Mat;
        if(curr_stage == Stage.ROI.ordinal()) {
//            ROI = new Mat(thresh, new Rect(pointA, pointB));
            Imgproc.resize(ROI, ROI, new Size(show_resolution.x, show_resolution.y));
            return ROI;
        }
        return thresh;
    }

    void determin_ring(Mat input){
        createMask(input, proc_Mat);
        //roi
        ROI = new Mat(input, new Rect(pointA, pointB));
        //resize
        Imgproc.resize(ROI, ROI, new Size(proc_resolution.x, proc_resolution.y));
        //blur
        Imgproc.blur(ROI, ROI, new Size(3,3));
        //create mask
        createMask(ROI, ROI);

        white_pixel = Core.countNonZero(ROI);
    }

    public void setHSVThreshold(int min_h,
                                int max_h,
                                int min_s,
                                int max_s,
                                int min_v,
                                int max_v){

        threshold  = new HSV_threshold(min_h, max_h, min_s, max_s, min_v, max_v);
    }

    public void setHSVThreshold(HSV_threshold threshold){

        this.threshold  = new HSV_threshold(threshold);
    }


    void createMask(Mat src, Mat dst){
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
        Core.inRange(dst,threshold.min(), threshold.max(), dst);
    }

}
