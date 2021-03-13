package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.IMU;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTrain;

@TeleOp(group = "Test")
public class WorldMecanumDrive extends OpMode {
    ElapsedTime timer = new ElapsedTime();
    MecanumDriveTrain driveTrain;

    @Override
    public void init() {
        driveTrain = new MecanumDriveTrain(hardwareMap, true);

    }

    double heading = 0;
    double last_heading = 0;
    boolean run = false;
    boolean last_a = false;
    double total_time = 0;
    @Override
    public void loop() {

        if(gamepad1.a && !last_a) run = !run;
        last_a = gamepad1.a;
        if(run){
//            double raw_x = gamepad1.left_stick_x;
//            double raw_y = gamepad1.left_stick_y;
//            double raw_r = gamepad1.right_stick_x;
            double raw_x = 0;
            double raw_y = 0;
            double raw_r = 0.5;
            heading = driveTrain.get_heading(AngleUnit.RADIANS);
            last_heading = heading;
            double x = raw_x * Math.cos(last_heading) - raw_y * Math.sin(last_heading);
            double y = -raw_x * Math.sin(last_heading) - raw_y * Math.cos(last_heading);
            double r = raw_r * 0.5;
            telemetry.addData("det_theta", (heading - last_heading) * 180 / Math.PI);
            x *= 0.5;
            y *= 0.7;
            total_time = timer.seconds();
            driveTrain.move(x, y, r);
            telemetry.addData("imu", heading);
            telemetry.addData("total_time", total_time);

            telemetry.update();
        }
        else{
            driveTrain.move(0, 0, 0);
            timer.reset();
        }


    }
}
