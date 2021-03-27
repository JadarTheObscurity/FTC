package org.firstinspires.ftc.teamcode.andrew;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;

@TeleOp(group = "Andrew")
@Config
public class AndrewDrive extends OpMode {
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    Andrew andrew;
    PID tower_pid = new PID(-0.5, 0,0 );
    @Override
    public void init() {
        andrew = new Andrew(hardwareMap);
        andrew.setStartPose(new Pose2d(0, 0, 0));
//        webcam = new Webcam(hardwareMap, new Point(640, 480));
//        webcam.setPipeline(pipeline);
//        webcam.startStreaming(Andrew.dashboard);
    }

    boolean want_to_suck = false,
    want_to_shoot = false,
    want_to_fire = false,
    want_to_grab = false;
    boolean last_x = false,
    last_left_bumper = false,
    last_right_bumper = false,
    last_y = false;

    public static double kp = -0.5;
    public static double ki = 0;
    public static double kd = 0;
    @Override
    public void loop() {
        double drivetrain_rotate = gamepad1.right_stick_x;
//        if(gamepad1.a && pipeline.found){
//            drivetrain_rotate = tower_pid.result(0.5, (double)pipeline.x / 640);
//        }
//        else tower_pid.reest();
        tower_pid.set(kp, ki, kd);
        andrew.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, drivetrain_rotate);

        Andrew.packet.put("tower_x", pipeline.x);
        Andrew.packet.put("tower_y", pipeline.y);
        Andrew.packet.put("tower found", pipeline.found);

        if(gamepad1.x && !last_x) {
            want_to_suck = !want_to_suck;
            if(want_to_suck) want_to_shoot = false;
        }
        if(gamepad1.y && !last_y) want_to_grab = !want_to_grab;
        if(gamepad1.left_bumper && !last_left_bumper) {
            want_to_shoot = !want_to_shoot;
            if(want_to_shoot) want_to_suck = false;
        }
        if(gamepad1.right_bumper && !last_right_bumper) andrew.shoot_timer.reset();

        last_x = gamepad1.x;
        last_y = gamepad1.y;
        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;

        if(andrew.shoot_timer.seconds() < andrew.shoot_duration/2) want_to_fire = true;
        else want_to_fire = false;

        if(want_to_shoot){
            andrew.shooter_shoot();
            andrew.load_up();
        }
        else{
            andrew.shooter_stop();
            andrew.load_down();
        }

        if(gamepad1.b) andrew.suck_reverse();
        else if(want_to_suck) andrew.suck_spin();
        else andrew.suck_stop();

        if(want_to_fire && want_to_shoot) andrew.fire_on();
        else andrew.fire_off();

        if(want_to_grab) andrew.claw_grab();
        else andrew.claw_release();
        andrew.update();
    }

    @Override
    public void stop() {
        andrew.finish();
    }
}
