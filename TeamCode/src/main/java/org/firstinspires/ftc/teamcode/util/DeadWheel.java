package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DeadWheel {
    DcMotorEx dw_x;
    DcMotorEx dw_y_l;
    DcMotorEx dw_y_r;



    public DeadWheel(DcMotorEx dw_x, DcMotorEx dw_y_l, DcMotorEx dw_y_r){
        this.dw_x = dw_x;
        this.dw_y_l = dw_y_l;
        this.dw_y_r = dw_y_r;
    }


}
