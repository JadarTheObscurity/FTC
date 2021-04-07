package org.firstinspires.ftc.teamcode.andrew;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.IMU;

@TeleOp(group = "Andrew")
@Disabled
public class ForwardOffest extends OpMode {
    Andrew andrew;
    IMU imu;
    DcMotorEx m_x;
    @Override
    public void init() {
        andrew = new Andrew(hardwareMap);
        m_x = hardwareMap.get(DcMotorEx.class, "rf");
        imu = new IMU(hardwareMap);
    }

    @Override
    public void loop() {
        if(imu.getHeading(AngleUnit.DEGREES) < 150){
            andrew.move(0, 0, -0.3);
        }
        else{
            andrew.move(0, 0, 0);
        }
        telemetry.addData("Pose", andrew.curr_pos);
        telemetry.addData("imu", imu.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    @Override
    public void stop() {
        andrew.finish();
    }
}
