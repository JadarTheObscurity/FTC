package org.firstinspires.ftc.teamcode.louis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Louis Speed Test", group = "Louis")
@Config
public class SpeedTest extends OpMode {
    Louis louis;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        louis = new Louis(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        timer.reset();
    }

    double start_x  =0;
    double end_x = 0;
    double start_y = 0;
    double end_y= 0;
    double power = 0.5;
//113 167
    int stage = 0;
    public static double speed = 30;
    @Override
    public void loop() {
        test();
    }

    void test(){
        if(stage == 0){
            if(timer.seconds() > 3){
                timer.reset();
//                louis.reset_pos();
                stage = 1;
                return;
            }

            if(timer.seconds() < 1.5) louis.move(power, 0, 0);
            else louis.move(0, 0, 0);

            if(timer.seconds() < 0.2) start_x = louis.driveTrain.get_x();
            if(timer.seconds() < 1.2) end_x = louis.driveTrain.get_x();
        }
        if(stage == 1){
            if(timer.seconds() < 1.5) louis.move(0, power, 0);
            else louis.move(0, 0, 0);

            if(timer.seconds() < 0.2) start_y = louis.driveTrain.get_y();
            if(timer.seconds() < 1.2) end_y = louis.driveTrain.get_y();
        }


        telemetry.addData("start x", start_x);
        telemetry.addData("end x", end_x);
        telemetry.addData("x power_speed_ratio", (end_x - start_x)/power);
        telemetry.addData("y power_speed_ratio", (end_y - start_y)/power);
        telemetry.update();
    }
}
