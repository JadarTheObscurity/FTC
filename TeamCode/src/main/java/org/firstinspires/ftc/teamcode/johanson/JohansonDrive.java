package org.firstinspires.ftc.teamcode.johanson;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;

@TeleOp(group = "Johanson")
@Config
@Disabled
public class JohansonDrive extends OpMode {
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    Johanson johanson;
    PID tower_pid = new PID(-0.5, 0,0 );
    @Override
    public void init() {
        johanson = new Johanson(hardwareMap);
        johanson.setStartPose(new Pose2d(0, 0, 0));
//        webcam = new Webcam(hardwareMap, new Point(640, 480));
//        webcam.setPipeline(pipeline);
//        webcam.startStreaming(Johanson.dashboard);
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
        if(gamepad1.a && pipeline.found){
            drivetrain_rotate = tower_pid.result(0.5, (double)pipeline.x / 640);
        }
        else tower_pid.reest();
        tower_pid.set(kp, ki, kd);
        johanson.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, drivetrain_rotate);

        Johanson.packet.put("tower_x", pipeline.x);
        Johanson.packet.put("tower_y", pipeline.y);
        Johanson.packet.put("tower found", pipeline.found);

        if(gamepad1.x && !last_x) {
            want_to_suck = !want_to_suck;
            if(want_to_suck) want_to_shoot = false;
        }
        if(gamepad1.y && !last_y) want_to_grab = !want_to_grab;
        if(gamepad1.left_bumper && !last_left_bumper) {
            want_to_shoot = !want_to_shoot;
            if(want_to_shoot) want_to_suck = false;
        }
        if(gamepad1.right_bumper && !last_right_bumper) johanson.shoot_timer.reset();

        last_x = gamepad1.x;
        last_y = gamepad1.y;
        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;

        if(johanson.shoot_timer.seconds() < johanson.shoot_duration/2) want_to_fire = true;
        else want_to_fire = false;

        if(want_to_shoot){
            johanson.shooter_shoot();
            johanson.load_up();
        }
        else{
            johanson.shooter_stop();
            johanson.load_down();
        }

        if(gamepad1.b) johanson.suck_reverse();
        else if(want_to_suck) johanson.suck_spin();
        else johanson.suck_stop();

        if(want_to_fire && want_to_shoot) johanson.fire_on();
        else johanson.fire_off();

        if(want_to_grab) johanson.claw_grab();
        else johanson.claw_release();
        johanson.update();
    }

    @Override
    public void stop() {
        johanson.finish();
    }
}
