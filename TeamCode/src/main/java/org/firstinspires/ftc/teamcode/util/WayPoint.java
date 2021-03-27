package org.firstinspires.ftc.teamcode.util;

public class WayPoint extends Pose2d{
    double time = -1;
    public WayPoint(double x, double y, double r) {
        super(x, y, r);
    }

    public WayPoint(double x, double y, double r, double time) {
        super(x, y, r);
        this.time = time;
    }

    public Pose2d getPose2d(){
        return new Pose2d(getX(), getY(), getR());
    }

    public double getTime(){
        return time;
    }
}
