package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    public static double x_kp = 0.07;
    public static double y_kp = 0.07;
    public static double r_kp = -0.03;
    public double time_mult = 1;

    public double x_target = 0;
    public double y_target = 0;
    public double r_target = 0;
    public double vx_target = 0;
    public double vy_target = 0;

    public double x_error = 0;
    public double y_error = 0;
    public double r_error = 0;


    public boolean moveTo_IMU(double x, double y, double r){
        double s = Math.hypot(x, y);
        //Calculate how long does it takes to reach to the target position
        double slide_time = max(s/2/ MecanumDriveTrain.max_v, sqrt(s/2/ MecanumDriveTrain.max_a));
        double turn_time = max(abs(clipAngleDegree(r,driveTrain.last_imu))/2/ MecanumDriveTrain.max_w, sqrt(abs(clipAngleDegree(r,driveTrain.last_imu))/2/ MecanumDriveTrain.max_alpha));
        double total_time = max(slide_time, turn_time);

        //find the error between the robot's current position and where it should be

        x_target = s_of_t(x, timer.seconds(), total_time);
        y_target = s_of_t(y, timer.seconds(), total_time);
        r_target = s_of_t(clipAngleDegree(r,driveTrain.last_imu), timer.seconds(), total_time);

        vx_target = v_of_t(x, timer.seconds(), total_time);
        vy_target = v_of_t(y, timer.seconds(), total_time);

        x_error = x_target - driveTrain.get_x();
        y_error = y_target - driveTrain.get_y();
        r_error = clipAngleDegree(r_target, clipAngleDegree(driveTrain.get_heading(AngleUnit.DEGREES),driveTrain.last_imu));

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

    public boolean moveTo_Encoder(double x, double y, double r){
        double s = Math.hypot(x, y);
        //Calculate how long does it takes to reach to the target position
        double slide_time = max(s/2/ MecanumDriveTrain.max_v, sqrt(s/2/ MecanumDriveTrain.max_a));
        double turn_time = max(abs(r)/2/ MecanumDriveTrain.max_w, sqrt(abs(r)/2/ MecanumDriveTrain.max_alpha));
        double total_time = max(slide_time, turn_time);
        //Nothing have to be done, so return
        if(total_time == 0) {
            driveTrain.move(0, 0,0);
            return true;
        }

        //find the error between the robot's current position and where it should be

        x_target = s_of_t(x, timer.seconds(), total_time);
        y_target = s_of_t(y, timer.seconds(), total_time);
        r_target = s_of_t(r, timer.seconds(), total_time);

        vx_target = v_of_t(x, timer.seconds(), total_time);
        vy_target = v_of_t(y, timer.seconds(), total_time);

        x_error = x_target - driveTrain.get_x();
        y_error = y_target - driveTrain.get_y();
        r_error = r_target - driveTrain.get_r();

        double power_limit_y = 1;
        double power_limit_x = 1;
        double power_limit_r = 1;


        if(timer.seconds() > total_time / 2){
            if(sameSign(y, y_error)) y_error = 0;
        }

        //P control
        double x_power = Range.clip(vx_target / driveTrain.x_pv_ratio + x_error * x_kp, -power_limit_x, power_limit_x);
        double y_power = Range.clip(vy_target / driveTrain.y_pv_ratio + y_error * y_kp, -power_limit_y, power_limit_y);
//        double x_power = Range.clip(x_error * x_kp, -power_limit_x, power_limit_x);
//        double y_power = Range.clip(y_error * y_kp, -power_limit_y, power_limit_y);
        double r_power = Range.clip(r_error * r_kp, -power_limit_r, power_limit_r);




        // add extra 0.1 second to yeah you know what I mean
        if(timer.seconds() > total_time +0.1 ){
            driveTrain.move(0, 0, 0);
            return true;
        }
        else driveTrain.move(x_power, y_power, r_power);
        return false;
    }

    public boolean moveTo_OW(Pose2d start, Pose2d end, Pose2d curr){
//        double s = Math.hypot(rel.getX(), rel.getY());
        //Calculate how long does it takes to reach to the target position
//        double slide_time = max(s/2/ MecanumDriveTrain.max_v, sqrt(s/2/ MecanumDriveTrain.max_a));
//        double turn_time = max(abs(rel.getR(AngleUnit.DEGREES))/2/ MecanumDriveTrain.max_w, sqrt(abs(rel.getR(AngleUnit.DEGREES))/2/ MecanumDriveTrain.max_alpha));
//        double total_time = max(slide_time, turn_time);

        double total_time = getTotalTime(start, end);

        total_time *= time_mult;

        //Nothing have to be done, so return
        if(total_time == 0) {
            driveTrain.move(0, 0,0);
            return true;
        }


        Pose2d rel = Pose2d.det_pose(start, end);

        //find the error between the robot's current position and where it should be)
        x_target = s_of_t(rel.getX(), timer.seconds(), total_time);
        y_target = s_of_t(rel.getY(), timer.seconds(), total_time);
        r_target = s_of_t(rel.getR(AngleUnit.DEGREES), timer.seconds(), total_time);

        vx_target = v_of_t(rel.getX(), timer.seconds(), total_time);
        vy_target = v_of_t(rel.getY(), timer.seconds(), total_time);

        Pose2d from_start = Pose2d.det_pose(start, curr);
        x_error = x_target - from_start.getX();
        y_error = y_target - from_start.getY();
        r_error = r_target - from_start.getR(AngleUnit.DEGREES);

        double power_limit_y = 1;
        double power_limit_x = 1;
        double power_limit_r = 1;

        double robot_x_error = x_error * Math.cos(curr.getR()) + y_error * Math.sin(curr.getR());
        double robot_y_error = -x_error * Math.sin(curr.getR()) + y_error * Math.cos(curr.getR());

        double robot_vx_target = vx_target * Math.cos(curr.getR()) + vy_target * Math.sin(curr.getR());
        double robot_vy_target = -vx_target * Math.sin(curr.getR()) + vy_target * Math.cos(curr.getR());


//        if(timer.seconds() > total_time / 2){
//            if(sameSign(y, y_error)) y_error = 0;
//        }

        //P control
//        double x_power = Range.clip(robot_vx_target / driveTrain.x_pv_ratio + robot_x_error * x_kp, -power_limit_x, power_limit_x);
//        double y_power = Range.clip(robot_vy_target / driveTrain.y_pv_ratio + robot_y_error * y_kp, -power_limit_x, power_limit_x);
        double x_power = Range.clip(robot_x_error * x_kp, -power_limit_x, power_limit_x);
        double y_power = Range.clip( robot_y_error * y_kp, -power_limit_y, power_limit_y);
        double r_power = Range.clip(r_error * r_kp, -power_limit_r, power_limit_r);




        // add extra 0.1 second to yeah you know what I mean
        if(timer.seconds() > total_time + 0.2 && Pose2d.det_pose(curr, end).getlength() <= 5){
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


    //TODO Replace it into ow
    double getTotalTime(Pose2d start, Pose2d end){
        Pose2d rel = Pose2d.det_pose(start, end);
        double s = Math.hypot(rel.getX(), rel.getY());
        //Calculate how long does it takes to reach to the target position
        double slide_time = max(s/2/ MecanumDriveTrain.max_v, sqrt(s/2/ MecanumDriveTrain.max_a));
        double turn_time = max(abs(rel.getR(AngleUnit.DEGREES))/2/ MecanumDriveTrain.max_w, sqrt(abs(rel.getR(AngleUnit.DEGREES))/2/ MecanumDriveTrain.max_alpha));
        double total_time = slide_time + turn_time;
        return total_time;
    }

    boolean sameSign(double a, double b){
        if(a == 0 || b == 0) return true;
        if(a * b < 0) return false;
        return true;
    }
}
