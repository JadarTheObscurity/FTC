package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Pose2d {
    private double x = 0;
    private double y = 0;
    private double r = 0;
    public Pose2d(double x, double y, double r){

        this.x = x;
        this.y = y;
        this.r = r;
    }
    public double getR() {
        return r;
    }
    public double getR(AngleUnit unit) {
        if(unit == AngleUnit.DEGREES) return Math.toDegrees(r);
        return r;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void set(double x, double y, double r){
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public void set(Pose2d pose2d){
        this.x = pose2d.getX();
        this.y = pose2d.getY();
        this.r = pose2d.getR();
    }

    public static Pose2d det_pose(Pose2d start, Pose2d end){
        return new Pose2d(end.getX() - start.getX(),
                end.getY() - start.getY(),
                end.getR() - start.getR());
    }

    public Vector2d headingVec(){
        return new Vector2d(Math.sin(r), Math.cos(r));
    }

    @Override
    public String toString(){
        return "x: " + (int)x + "\ty: "+(int)y+"\tr: "+(int)Math.toDegrees(r);
    }
}
