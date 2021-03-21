package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Scalar;

public class HSV_threshold{
    public int min_h;
    public int max_h;
    public int min_s;
    public int max_s;
    public int min_v;
    public int max_v;

    public HSV_threshold(int min_h,int max_h,int min_s,int max_s,int min_v,int max_v){
        this.min_h = min_h;
        this.max_h = max_h;
        this.min_s = min_s;
        this.max_s = max_s;
        this.min_v = min_v;
        this.max_v = max_v;
    }

    public HSV_threshold(HSV_threshold threshold){
        this(threshold.min_h, threshold.max_h, threshold.min_s, threshold.max_s, threshold.min_v, threshold.max_v);
    }

    public Scalar min(){
        return new Scalar(min_h, min_s, min_v);
    }

    public Scalar max(){
        return new Scalar(max_h, max_s, max_v);
    }

    public HSV_threshold copy(){
        return new HSV_threshold(this);
    }
}
