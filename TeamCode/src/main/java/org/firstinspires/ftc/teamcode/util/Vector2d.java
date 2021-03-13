package org.firstinspires.ftc.teamcode.util;

public class Vector2d {
    double x = 0;
    double y = 0;

    public Vector2d(double x, double y){

        this.x = x;
        this.y = y;
    }

    public Vector2d times(double ratio){
        return new Vector2d(x * ratio, y * ratio);
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }
}
