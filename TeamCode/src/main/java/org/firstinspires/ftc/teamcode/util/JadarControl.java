package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

@Config
public class JadarControl {
    MecanumDriveTrain driveTrain;
    ElapsedTime timer = new ElapsedTime();

    public JadarControl(MecanumDriveTrain driveTrain){
        this.driveTrain = driveTrain;
        reset();
    }

    public void reset(){
        timer.reset();
    }

    public static double x_kp = 0.1;
    public static double y_kp = 0.1;
    public static double r_kp = -0.03;

    public double x_target = 0;
    public double y_target = 0;
    public double r_target = 0;
    public double vx_target = 0;
    public double vy_target = 0;


    public boolean moveTo(double x, double y, double r){
        double s = Math.hypot(x, y);
        //Calculate how long does it takes to reach to the target position
        double slide_time = max(s/2/ MecanumDriveTrain.max_v, sqrt(s/2/ MecanumDriveTrain.max_a));
        double turn_time = max(abs(r-driveTrain.last_imu)/2/ MecanumDriveTrain.max_w, sqrt(abs(r-driveTrain.last_imu)/2/ MecanumDriveTrain.max_alpha));
        double total_time = max(slide_time, turn_time);

        //find the error between the robot's current position and where it should be

        x_target = s_of_t(x, timer.seconds(), total_time);
        y_target = s_of_t(y, timer.seconds(), total_time);
        r_target = s_of_t(r-driveTrain.last_imu, timer.seconds(), total_time);

        vx_target = v_of_t(x, timer.seconds(), total_time);
        vy_target = v_of_t(y, timer.seconds(), total_time);

        double x_error = x_target - driveTrain.get_x();
        double y_error = y_target - driveTrain.get_y();
        double r_error = clipAngleDegree(r_target, driveTrain.get_heading()-driveTrain.last_imu);

        //P control

        double power_limit = 1;
        double x_power = vx_target / driveTrain.x_pv_ratio + Range.clip(x_error * x_kp, -power_limit, power_limit);
        double y_power = vy_target / driveTrain.y_pv_ratio + Range.clip(y_error * y_kp, -power_limit, power_limit);
//        double x_power = Range.clip(x_error * x_kp, -power_limit, power_limit);
//        double y_power = Range.clip(y_error * y_kp, -power_limit, power_limit);
        double r_power = Range.clip(r_error * r_kp, -power_limit, power_limit);




        // add extra 0.1 second to yeah you know what I mean
        if(timer.seconds() > total_time + 0.1){
            driveTrain.move(0, 0, 0);
            return true;
        }
        else driveTrain.move(x_power, y_power, r_power);
        return false;
    }

    double s_of_t(double s, double t, double total_time){
        if(t <= total_time)
            return -s/2 * Math.cos(t / total_time * 3.1415 ) + s/2;
        return s;
    }

    double v_of_t(double s, double t, double total_time){
        if(t <= total_time){
            return s / total_time / 2 * Math.PI * Math.sin(t / total_time * Math.PI);
        }
        return 0;
    }

    double clipAngleDegree(double a, double b){
        double diff = a - b;
        while(diff > 180) diff -= 360;
        while(diff < -180) diff += 360;
        return diff;
    }

}
