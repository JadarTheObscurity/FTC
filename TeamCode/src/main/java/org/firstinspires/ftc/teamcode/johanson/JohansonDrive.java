package org.firstinspires.ftc.teamcode.johanson;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

@TeleOp(group = "Johanson")
public class JohansonDrive extends Johanson{
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    @Override
    public void init() {
        super.init();
        setStartPose(new Pose2d(0, 0, 0));
        webcam = new Webcam(hardwareMap, new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.startStreaming(dashboard);
    }

    boolean want_to_suck = false,
    want_to_shoot = false,
    want_to_fire = false,
    want_to_grab = false;
    boolean last_x = false,
    last_left_bumper = false,
    last_right_bumper = false,
    last_y = false;

    public static double auto_aim_kp = 0.5;
    @Override
    public void loop() {
        double drivetrain_rotate = gamepad1.right_stick_x;
        if(gamepad1.a && pipeline.found){
            drivetrain_rotate = (double)(pipeline.x - 320) / 640 * auto_aim_kp;
        }
        move(gamepad1.left_stick_x, -gamepad1.left_stick_y, drivetrain_rotate);

        packet.put("tower_x", pipeline.x);
        packet.put("tower_y", pipeline.y);
        packet.put("tower found", pipeline.found);

        if(gamepad1.x && !last_x) {
            want_to_suck = !want_to_suck;
            if(want_to_suck) want_to_shoot = false;
        }
        if(gamepad1.y && !last_y) want_to_grab = !want_to_grab;
        if(gamepad1.left_bumper && !last_left_bumper) {
            want_to_shoot = !want_to_shoot;
            if(want_to_shoot) want_to_suck = false;
        }
        if(gamepad1.right_bumper && !last_right_bumper) shoot_timer.reset();

        last_x = gamepad1.x;
        last_y = gamepad1.y;
        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;

        if(shoot_timer.seconds() < shoot_duration/2) want_to_fire = true;
        else want_to_fire = false;

        if(want_to_shoot){
            shooter_shoot();
            load_up();
        }
        else{
            shooter_stop();
            load_down();
        }

        if(gamepad1.b) suck_reverse();
        else if(want_to_suck) suck_spin();
        else suck_stop();

        if(want_to_fire && want_to_shoot) fire_on();
        else fire_off();

        if(want_to_grab) claw_grab();
        else claw_release();
        super.loop();
    }
}
