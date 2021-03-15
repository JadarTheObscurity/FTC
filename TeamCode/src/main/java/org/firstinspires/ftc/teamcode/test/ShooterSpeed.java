package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Test")
@Disabled
public class ShooterSpeed extends OpMode {
    DcMotorEx shoot;

    @Override
    public void init() {
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        shoot.setVelocity(-1800);
        telemetry.addData("shoot speed", shoot.getVelocity());
        telemetry.update();
    }
}
