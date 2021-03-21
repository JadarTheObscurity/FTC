package org.firstinspires.ftc.teamcode.test.odometrywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.IMU;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTrain;

@TeleOp(group = "test")
public class LateralDistance extends OpMode {
    IMU imu;
    DcMotorEx ow_x;
    MecanumDriveTrain driveTrain;
    @Override
    public void init() {
        driveTrain = new MecanumDriveTrain(hardwareMap, false);

        ow_x = hardwareMap.get(DcMotorEx.class, "rf");
        ow_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ow_x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = new IMU(hardwareMap);
        imu.init();
    }

    @Override
    public void loop() {
        double heading = imu.getHeading(AngleUnit.DEGREES);
        if(heading < 150) driveTrain.move(0, 0, -0.2);
        else driveTrain.move(0,0,0);

        double x_value = ow_x.getCurrentPosition();

        telemetry.addData("heading", heading);
        telemetry.addData("x", x_value);
        telemetry.addData("ratio", x_value / heading / 180 * Math.PI);
        telemetry.update();
    }
}
