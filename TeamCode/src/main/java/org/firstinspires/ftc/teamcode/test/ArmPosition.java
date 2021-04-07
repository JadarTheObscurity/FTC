package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Test")
@Disabled
public class ArmPosition extends OpMode {
    DcMotorEx arm;
    @Override
    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }

    @Override
    public void loop() {
        telemetry.addData("Position", arm.getCurrentPosition());
        telemetry.update();
    }
}
