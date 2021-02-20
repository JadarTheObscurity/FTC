package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class MecanumDriveTrain {
    DcMotorEx lf, lb, rf, rb;
    IMU imu;
    double[] last_pos = {0, 0, 0, 0};
    public double last_imu;
    ArrayList<DcMotorEx> motors;

    public  final double
            max_v = 50,
            max_a = 25,
            max_w = 30,
            max_alpha=20;

    public MecanumDriveTrain(HardwareMap hardwareMap, boolean use_encoder){
        lf = hardwareMap.get(DcMotorEx.class, "lf"); //0
        lb = hardwareMap.get(DcMotorEx.class, "lb"); //1
        rf = hardwareMap.get(DcMotorEx.class, "rf"); //2
        rb = hardwareMap.get(DcMotorEx.class, "rb"); //3

        motors.add(lf);motors.add(lb);motors.add(rf);motors.add(rb);

        if(use_encoder) {
            for (DcMotorEx m : motors){
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        else{
            for (DcMotorEx m : motors)
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = new IMU(hardwareMap);
        imu.init();

        reset_pos();
    }

    public void move(double x, double y, double r){
        lf.setPower(x + y + r);
        lb.setPower(-x + y + r);
        rf.setPower(x - y + r);
        rb.setPower(-x - y + r);
    }

    double motor_tpr = 537.6;
    double wheel_diameter =10.16;


    public double get_x(){
        double raw_x =  motors.get(0).getCurrentPosition() - last_pos[0] -
                        motors.get(1).getCurrentPosition() - last_pos[1] +
                        motors.get(2).getCurrentPosition() - last_pos[2] -
                        motors.get(3).getCurrentPosition() - last_pos[3];
        return raw_x / 4 * 2 * Math.PI / motor_tpr * wheel_diameter / 2;
    }

    public double get_y(){
        double raw_y =  motors.get(0).getCurrentPosition() - last_pos[0] +
                        motors.get(1).getCurrentPosition() - last_pos[1] -
                        motors.get(2).getCurrentPosition() - last_pos[2] -
                        motors.get(3).getCurrentPosition() - last_pos[3];
        return raw_y / 4 * 2 * Math.PI / motor_tpr * wheel_diameter / 2;
    }

    public double get_heading(){
        return imu.getHeading();
    }

    public void reset_pos(){
        //reset motors
        for(int i = 0; i < 4; i++){
            last_pos[i] = motors.get(i).getCurrentPosition();
        }
        //reset imu
        last_imu = get_heading();
    }

}
