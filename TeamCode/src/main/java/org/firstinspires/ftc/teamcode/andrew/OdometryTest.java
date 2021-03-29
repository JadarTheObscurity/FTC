package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.IMU;

@TeleOp(group = "Andrew")
public class OdometryTest extends OpMode {
    Andrew andrew;
    IMU imu;
    @Override
    public void init() {
        andrew = new Andrew(hardwareMap);
        imu = new IMU(hardwareMap);
    }

    @Override
    public void loop() {
        double heading = imu.getHeading(AngleUnit.DEGREES);
        if(andrew.curr_pos.getR() > -Math.toRadians(1080)){
            andrew.move(0, 0.5, 0.3);
        }
        else {
            andrew.move(0, 0, 0);
        }
        andrew.put_packet("imu", heading);
        andrew.update();
    }

    @Override
    public void stop() {
        andrew.finish();
    }
}
