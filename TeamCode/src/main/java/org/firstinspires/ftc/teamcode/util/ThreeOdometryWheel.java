package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

public class ThreeOdometryWheel {
    DcMotorEx ly, ry, x;
    ArrayList<DcMotorEx> encoders = new ArrayList<>();
    public static double[] last_deadwheel_encoder = {0, 0, 0};


    public ThreeOdometryWheel(DcMotorEx ly, DcMotorEx ry, DcMotorEx x){
        this.ly = ly;
        this.ry = ry;
        this.x = x;

        encoders.add(ly);
        encoders.add(ry);
        encoders.add(x);

        reset();
    }

    public void reset(){
        for (int i = 0; i < 3; i++) {
            last_deadwheel_encoder[i] = encoders.get(i).getCurrentPosition();
        }
    }


}
