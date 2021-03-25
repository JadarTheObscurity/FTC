package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "Test")
@Disabled
public class PureOdometry extends LinearOpMode {
    static DcMotorEx lf, lb, rf, rb;
    static ArrayList<DcMotorEx> motors = new ArrayList<>();
    static List<LynxModule> allHubs;
    static double[] last_pos = {0, 0, 0, 0};
    static Pose2d curr_pos = new Pose2d(0, 0, 0);
    static OdometryThread odometryThread = new OdometryThread();

    @Override
    public void runOpMode() throws InterruptedException {
        setupMotors();
        odometryThread.start();
        waitForStart();

        odometryThread.interrupt();
    }

    void setupMotors(){
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        allHubs = hardwareMap.getAll(LynxModule.class);
        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        for(DcMotorEx m : motors){
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        for(LynxModule module : allHubs){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        reset_pos();
    }

    static double motor_tpr = 537.6;
    static double wheel_diameter =10.16;
    static double robot_width = 45;
    static double tpr_to_cm = 2 * Math.PI / motor_tpr * wheel_diameter / 2;

    static double x_ratio = 1;
    static double y_ratio = 1;
    static double r_ratio = 1;
    static double get_det_x(){
        double raw_x =  (motors.get(0).getCurrentPosition() - last_pos[0]) -
                (motors.get(1).getCurrentPosition() - last_pos[1]) +
                (motors.get(2).getCurrentPosition() - last_pos[2]) -
                (motors.get(3).getCurrentPosition() - last_pos[3]);
        return raw_x / 4 * tpr_to_cm * x_ratio;
    }

    static double get_det_y(){
        double raw_y =  (motors.get(0).getCurrentPosition() - last_pos[0]) +
                (motors.get(1).getCurrentPosition() - last_pos[1]) -
                (motors.get(2).getCurrentPosition() - last_pos[2]) -
                (motors.get(3).getCurrentPosition() - last_pos[3]);
        return raw_y / 4 * tpr_to_cm * y_ratio;
    }

    static double get_det_r(){
        double raw_r =  (motors.get(0).getCurrentPosition() - last_pos[0]) +
                (motors.get(1).getCurrentPosition() - last_pos[1]) +
                (motors.get(2).getCurrentPosition() - last_pos[2]) +
                (motors.get(3).getCurrentPosition() - last_pos[3]);
        return raw_r / 4 * tpr_to_cm / robot_width * 1.414 * r_ratio;
    }

    static void updateCoordinate(){
        double dr = get_det_r();
        double r = curr_pos.getR();
        double dx = Math.cos(r + dr/2) * get_det_x() - Math.sin(r + dr/2) * get_det_y();
        double dy = Math.sin(r + dr/2) * get_det_x() + Math.cos(r + dr/2) * get_det_y();

        curr_pos.add(dx, dy, dr);
        reset_pos();
    }

    static void reset_pos(){
        //reset motors
        for(int i = 0; i < 4; i++){
            last_pos[i] = motors.get(i).getCurrentPosition();
        }
    }

    static class OdometryThread extends Thread{
        public void run(){
            try
            {
                while (!isInterrupted())
                {
                    for(LynxModule module : allHubs) module.clearBulkCache();
                    updateCoordinate();
                    Thread.sleep(10);
                }
            }
            catch (InterruptedException e) {System.out.println(e.toString());}
        }
    }

    static class Pose2d{
        double x;
        double y;
        double r;

        public Pose2d(double x, double y, double r){
            this.x = x;
            this.y = y;
            this.r = r;
        }

        public void add(double dx, double dy, double dr){
            x += dx;
            y += dy;
            r += dr;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getR() {
            return r;
        }
    }
}
