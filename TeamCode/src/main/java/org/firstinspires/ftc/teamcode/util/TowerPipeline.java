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
    Point process_resolution = new Point(160, 120);


    Mat thresh = new Mat();
    Mat ROI = new Mat();
    Mat show_HSV = new Mat();
    Mat proc_HSV = new Mat();
    Mat show = new Mat();
    Mat origin = new Mat();
    public int min_h = 10;
    public int max_h = 30;
    public int min_s = 30;
    public int max_s = 255;
    public int min_v = 200;
    public int max_v = 255;
    int min_area = 2000;

    public boolean found = false;
    public int x;
    public int y;

    Point pointA = new Point(0 * show_resolution.x / 8, 0 * show_resolution.y/8);
    Point pointB = new Point(8 * show_resolution.x / 8, 4 * show_resolution.y/8);

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

    private void toHSV(Mat input){
        Imgproc.cvtColor(input, show_HSV, Imgproc.COLOR_BGR2HSV);
    }

    @Override
    public void init(Mat input){
        toHSV(input);
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
        show = input.clone();
        Imgproc.blur(input, input, new Size(3, 3));
        toHSV(input);
        Core.inRange(show_HSV, new Scalar(min_h, min_s, min_v), new Scalar(max_h, max_s, max_v), thresh);

        findTower(show);


        if(curr_stage >= Stage.values().length) curr_stage = 0;
        if(curr_stage == Stage.Origin.ordinal()) {
            return show;
        }
        if(curr_stage == Stage.Thresh.ordinal()) {
            return  thresh;
        }
        if(curr_stage == Stage.ROI.ordinal()) {
            return show;
        }
        return thresh;
    }

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
    void findTower(Mat show){
        ROI = new Mat(thresh, new Rect(pointA, pointB));
        //find contour
        contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(ROI, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw contour
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
                Imgproc.rectangle(show, boundRect[i].tl(), boundRect[i].br(), new Scalar(173, 56, 237), 2);
            }
        }
        found = tem_found;

        if(found){
            x = (int)(boundRect[index].tl().x + boundRect[index].br().x)/2;
            y = (int)(boundRect[index].tl().y + boundRect[index].br().y)/2;
            Imgproc.line(show, new Point(x, 0), new Point(x, show.size().height),  new Scalar(0, 255, 255), 2);
            Imgproc.rectangle(show, boundRect[index].tl(), boundRect[index].br(), new Scalar(0, 255, 0), 3);
        }


    }


    public void setHSVThreshold(int min_y,
                                int max_y,
                                int min_s,
                                int max_s,
                                int min_v,
                                int max_v){

        this.min_h = min_y;
        this.max_h = max_y;
        this.min_s = min_s;
        this.max_s = max_s;
        this.min_v = min_v;
        this.max_v = max_v;
    }
}
