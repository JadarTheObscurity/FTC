package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Test")
public class Test extends OpMode {
    Servo arm;
    @Override
    public void init() {
        arm = hardwareMap.get(Servo.class, "arm");
    }

    @Override
    public void loop() {
        if(gamepad1.x) arm.setPosition(0.5);
        else arm.setPosition(0);
    }
}
