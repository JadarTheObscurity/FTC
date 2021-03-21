package org.firstinspires.ftc.teamcode.lucus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp(name="Lucus Drive", group = "Lucus")
@Disabled
public class LucusDrive extends OpMode {
    Lucus lucus;
    @Override
    public void init() {

        lucus = new Lucus(hardwareMap);
        lucus.setStartPose(new Pose2d(-150, -150, Math.toRadians(180)));
    }

    @Override
    public void loop() {
        lucus.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        lucus.update();
    }

    @Override
    public void stop(){
        lucus.finish();
    }
}
