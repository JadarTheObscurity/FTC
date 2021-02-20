package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;

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

    boolean moveTo(double x, double y, double r){
        double s = Math.hypot(x, y);
        //Calculate how long does it takes to reach to the target position
        double slide_time = max(s/2/driveTrain.max_v, sqrt(s/2/driveTrain.max_a));
        double turn_time = max(abs(r-driveTrain.last_imu)/2/driveTrain.max_w, sqrt(abs(r-driveTrain.last_imu)/2/driveTrain.max_w));
        double total_time = max(slide_time, turn_time);

        //find the error between the robot's current position and where it should be
        double x_error = cos_motion_profile_function(x, timer.seconds(), total_time) - driveTrain.get_x();
        double y_error = cos_motion_profile_function(y, timer.seconds(), total_time) - driveTrain.get_y();
        double r_error = cos_motion_profile_function(r-driveTrain.last_imu, timer.seconds(), total_time) - driveTrain.get_heading()-driveTrain.last_imu;

        //P control
        double x_kp = 0.1;
        double y_kp = 0.1;
        double r_kp = -0.03;
        double power_limit = 1;
        double x_power = Range.clip(x_error * x_kp, -power_limit, power_limit);
        double y_power = Range.clip(y_error * y_kp, -power_limit, power_limit);
        double r_power = Range.clip(r_error * r_kp, -power_limit, power_limit);


        // add extra 0.1 second to yeah you know what I mean
        if(timer.seconds() > total_time + 0.1){
            driveTrain.move(0, 0, 0);
            return true;
        }
        else driveTrain.move(x_power, y_power, r_power);
        return false;
    }

    double cos_motion_profile_function(double s, double t, double total_time){
        if(t <= total_time)
            return -s/2 * Math.cos(t / total_time * 3.1415 ) + s/2;
        return s;
    }

}
