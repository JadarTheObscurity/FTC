package org.firstinspires.ftc.teamcode.louis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Louis Drive", group = "Louis")
@Config
public class LouisDrive extends OpMode {
    Louis louis;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    DcMotorEx arm;
    Servo claw;
    ElapsedTime shoot_timer = new ElapsedTime();

    @Override
    public void init() {
        louis = new Louis(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
    }

    boolean grab = false;
    boolean last_y = false,
    last_left_bumper = false,
    last_x = false;
    boolean load = false,
    suck = false,
    last_suck = false,
    fire = false;
    public static double bias = 50;
    boolean fire_cooldwon_complete = false;
    @Override
    public void loop() {
        if(load && louis.see_tower())
            louis.move(gamepad1.left_stick_x*0.8, -gamepad1.left_stick_y*0.6, Range.clip((louis.tower_x()+bias)*0.002, -0.5, 0.5));
        else louis.move(gamepad1.left_stick_x*0.8, -gamepad1.left_stick_y*0.6, gamepad1.right_stick_x*0.5);
        if(gamepad1.a) louis.reset_pos();
        for(int i = 0; i < 4; i++) {
            packet.put("curr" + i + " ", louis.driveTrain.motors.get(i).getCurrentPosition());
            packet.put("last " + i + " ", louis.driveTrain.last_pos[i]);
        }

//suck
        if(gamepad1.x && !last_x) {
            suck = !suck;
            last_suck = suck;
            if(suck) load = false;
        }
        last_x = gamepad1.x;

        /**
         * @param !fire : don't do anything when the push servo is moving
         */
        if(gamepad1.left_bumper&& !last_left_bumper) {
            load = !load;
            if(load) suck = false;
            if(!load) suck = last_suck;
        }
        last_left_bumper = gamepad1.left_bumper;

        if(gamepad1.right_bumper && load && fire_cooldwon_complete) shoot_timer.reset();
        fire = shoot_timer.seconds() < 0.5;
        fire_cooldwon_complete = shoot_timer.seconds() > 0.6;


//
        if (gamepad1.b){
           louis.suck_reverse();
        }
        else if(suck){
           louis.suck_spin();
        }
        else {
            louis.suck_stop();
        }

        if(fire && load) louis.fire_on();
        else louis.fire_off();

        if(load || !fire_cooldwon_complete){
            louis.shooter_shoot();
        }
        else{
            louis.shooter_stop();
        }

        if(gamepad2.y && !last_y) grab = !grab;
        last_y = gamepad2.y;
        if(grab) louis.claw_grab();
        else louis.claw_release();

        if(gamepad2.b) louis.arm_down();
        else if(gamepad2.x)louis.arm_up();
        else louis.arm_stop();




        telemetry.addData("arm curent", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("r", louis.driveTrain.get_r());
        telemetry.addData("shoot speed", louis.shoot.getVelocity());
        packet.put("x", louis.driveTrain.get_x());
        packet.put("y", louis.driveTrain.get_y());
        packet.put("imu", louis.driveTrain.get_heading(AngleUnit.DEGREES));
        dashboard.sendTelemetryPacket(packet);
    }


}
