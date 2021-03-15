package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TowerPipeline extends OpenCvPipeline {

    Point show_resolution = new Point(640, 480);
    Point proc_resolution = new Point(160, 120);
    double x_ratio = show_resolution.x / proc_resolution.x;
    double y_ratio = show_resolution.y / proc_resolution.y;


    Mat ROI = new Mat();
    Mat show_HSV = new Mat();
    Mat proc_Mat = new Mat();
    Mat display_Mat = new Mat();
    Mat show_mat = new Mat();
    public int min_h = 100;
    public int max_h = 130;
    public int min_s = 30;
    public int max_s = 255;
    public int min_v = 200;
    public int max_v = 255;

    public TowerPipeline(Tower tower){
        switch (tower){
            case Blue:
                curr_hsv = blue_hsv;
                break;
            case Red:
                curr_hsv = red_hsv;
        }
    }

    public enum Tower{
        Blue,
        Red
    }

    final HSV_threshold blue_hsv = new HSV_threshold(100, 130, 30, 255, 200, 255);
    final HSV_threshold red_hsv = new HSV_threshold(0, 255, 0, 255, 0, 255);

    HSV_threshold curr_hsv;

    int min_area = 500;

    public boolean found = false;
    public int x;
    public int y;

    Point pointA = new Point(0 * proc_resolution.x / 8, 0 * proc_resolution.y/8);
    Point pointB = new Point(8 * proc_resolution.x / 8, 4 * proc_resolution.y/8);

    enum Stage{
        Origin,
        Mask,
        Debug
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

        Imgproc.resize(input, input, new Size(show_resolution.x, show_resolution.y));
        input.copyTo(show_mat);
        createMask(input, show_HSV);
        findTower(input);


        if(curr_stage >= Stage.values().length) curr_stage = 0;
        if(curr_stage == Stage.Origin.ordinal()) {
            show_mat.copyTo(display_Mat);
        }
        if(curr_stage == Stage.Mask.ordinal()) {

            show_HSV.copyTo(display_Mat);
        }
        if(curr_stage == Stage.Debug.ordinal()) {
            show_mat.copyTo(display_Mat);
        }
        if(display_Mat == null) input.copyTo(display_Mat);




        //release mat
//        input.release();
//        show_mat.release();
//        show_HSV.release();
//        proc_Mat.release();
//        ROI.release();

        return display_Mat;
    }

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
    void findTower(Mat input){
        //resize
        Imgproc.resize(input, proc_Mat, new Size(proc_resolution.x, proc_resolution.y));
        //blur
        Imgproc.blur(proc_Mat, proc_Mat, new Size(3,3));
        //create mask
        createMask(proc_Mat, proc_Mat);
        //roi
        ROI = new Mat(proc_Mat, new Rect(pointA, pointB));
        //find contour
        contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(ROI, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //find contour rectangle
        contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }


        //draw rectangle
        double max_area = 0;
        int index = 0;
        boolean tem_found = false;
        for (int i = 0; i < contours.size(); i++) {
            double area = boundRect[i].area();
            if(area > min_area) {
                if (area > max_area) {
                    tem_found = true;
                    index = i;
                }
                Imgproc.rectangle(show_mat,new Point(boundRect[i].tl().x * x_ratio, boundRect[i].tl().y * y_ratio),
                        new Point(boundRect[i].br().x * x_ratio, boundRect[i].br().y * y_ratio), new Scalar(173, 56, 237), 2);
            }
        }
        found = tem_found;

        if(found){
            x = (int)((boundRect[index].tl().x + boundRect[index].br().x)/2 * x_ratio);
            y = (int)((boundRect[index].tl().y + boundRect[index].br().y)/2 * y_ratio);
            Imgproc.line(show_mat, new Point(x, 0), new Point(x, show_mat.size().height),  new Scalar(0, 255, 255), 2);
            Imgproc.rectangle(show_mat,new Point(boundRect[index].tl().x * x_ratio, boundRect[index].tl().y * y_ratio),
                    new Point(boundRect[index].br().x * x_ratio, boundRect[index].br().y * y_ratio), new Scalar(0, 255, 0), 3);
        }


    }


    public void setHSVThreshold(int min_h,
                                int max_h,
                                int min_s,
                                int max_s,
                                int min_v,
                                int max_v){

       curr_hsv = new HSV_threshold(min_h, max_h, min_s, max_s, min_v, max_v);
    }

    void createMask(Mat src, Mat dst){
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);
        Core.inRange(dst,curr_hsv.min(), curr_hsv.max(), dst);
    }
}

