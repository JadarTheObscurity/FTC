package org.firstinspires.ftc.teamcode.util;

public class PID {
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double last_input = 0;
    double error_sum = 0;

    public PID(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double result(double target, double input){
        double error = target - input;
        error_sum += error;
        double det_input = input - last_input;
        last_input = input;
        return kp * error + ki * error_sum + kd * (last_input);
    }

    public void reest(){
        error_sum = 0;
    }
}
