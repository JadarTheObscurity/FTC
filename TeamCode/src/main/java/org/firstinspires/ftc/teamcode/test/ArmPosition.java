package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "Test")
@Disabled
public class ArmPosition extends OpMode {
    DcMotorEx arm;
    @Override
    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(1200);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            arm.setTargetPosition(1200);
            arm.setPower(0.3);
        }
        else if(gamepad1.b){
            arm.setTargetPosition(0);
            arm.setPower(0.3);
        }
//        else arm.setPower(0);
        telemetry.addData("Position", arm.getCurrentPosition());
        telemetry.update();
    }
}
