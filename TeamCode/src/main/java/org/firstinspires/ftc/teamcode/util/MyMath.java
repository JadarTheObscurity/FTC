package org.firstinspires.ftc.teamcode.util;

public class MyMath {
    public static double clipAngleDegree(double angle){
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    public static double clipAngleRadian(double angle){
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
