package org.firstinspires.ftc.teamcode.louis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.JadarControl;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

@Autonomous(name = "Louis Auto 1", group = "Louis")
@Disabled
public class LouisAuto1 extends OpMode {
    Louis louis;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    DcMotorEx arm;
    Servo claw;

    @Override
    public void init() {
        louis = new Louis(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        telemetry.addData("stage", louis.stage);
        telemetry.update();
        setup_trajectory();

        louis.claw_grab();
    }


    enum A_Stage {
        Start,
        move_to_shoot,
        shoot,
        put_fist_wobble,
        release_1,
        grab_second_wobble,
        grab,
        put_second_wobble,
        release_2,
        End
    }
    enum B_Stage {
        Start,
        move_to_shoot,
        shoot,
        put_fist_wobble,
        release_1,
        grab_second_wobble,
        grab,
        put_second_wobble,
        release_2,
        End
    }
    enum C_Stage {
        Start,
        move_to_shoot,
        shoot,
        put_fist_wobble,
        release_1,
        move_to_shoot_2,
        shoot_2,
        grab_second_wobble,
        grab,
        put_second_wobble,
        release_2,
        go_line,
        End
    }
//
//    ArrayList<Pose> put_first_wobble = new ArrayList<>();
//    ArrayList<Pose> release_1 = new ArrayList<>();
//    ArrayList<Pose> move_to_shoot = new ArrayList<>();
//    ArrayList<Pose> grab_second_wobble = new ArrayList<>();
//    ArrayList<Pose> put_second_wobble = new ArrayList<>();

    ArrayList<Pose> A_put_first_wobble = new ArrayList<>();
    ArrayList<Pose> A_release_1 = new ArrayList<>();
    ArrayList<Pose> A_move_to_shoot = new ArrayList<>();
    ArrayList<Pose> A_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose> A_put_second_wobble = new ArrayList<>();

    ArrayList<Pose> B_put_first_wobble = new ArrayList<>();
    ArrayList<Pose> B_release_1 = new ArrayList<>();
    ArrayList<Pose> B_move_to_shoot = new ArrayList<>();
    ArrayList<Pose> B_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose> B_put_second_wobble = new ArrayList<>();

    ArrayList<Pose> C_put_first_wobble = new ArrayList<>();
    ArrayList<Pose> C_release_1 = new ArrayList<>();
    ArrayList<Pose> C_move_to_shoot = new ArrayList<>();
    ArrayList<Pose> C_move_to_shoot_2 = new ArrayList<>();
    ArrayList<Pose> C_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose> C_put_second_wobble = new ArrayList<>();
    ArrayList<Pose> C_go_line = new ArrayList<>();

    @Override
    public void loop() {
        telemetry.addData("Curr Stage", louis.stage);
        telemetry.update();
        packet.put("x", louis.driveTrain.get_x());
        packet.put("y", louis.driveTrain.get_y());
        packet.put("target x", louis.control.x_target);
        packet.put("target y", louis.control.y_target);
        packet.put("error y", louis.control.y_error);
        packet.put("error y * kp", louis.control.y_error * JadarControl.y_kp);
        dashboard.sendTelemetryPacket(packet);

        C_state_machine();
    }

    void A_state_machine(){
        if (louis.stage == A_Stage.Start.ordinal()) {
            louis.nextStage();
        }
        else if (louis.stage == A_Stage.put_fist_wobble.ordinal()) {
            if (louis.moveTo(A_put_first_wobble)) louis.nextStage();
        }
        else if(louis.stage == A_Stage.release_1.ordinal()){
            louis.claw_release();
            louis.nextStage();
        }
        else if (louis.stage == A_Stage.move_to_shoot.ordinal()) {
            if (louis.moveTo(A_move_to_shoot)) louis.nextStage();
            louis.arm_down();
            louis.shooter_shoot();
        }
        else if (louis.stage == A_Stage.shoot.ordinal()) {
            if(louis.shoot_3_ring()) louis.nextStage();
        }
        else if (louis.stage == A_Stage.grab_second_wobble.ordinal()) {
            if (louis.moveTo(A_grab_second_wobble)) louis.nextStage();
        }
        else if(louis.stage == A_Stage.grab.ordinal()){
            if(louis.stage_timer.seconds() >= 1.5) louis.nextStage();
            if(louis.stage_timer.seconds() >= 0.4) louis.arm_up();
            louis.claw_grab();
        }
        else if (louis.stage == A_Stage.put_second_wobble.ordinal()) {
            if (louis.moveTo(A_put_second_wobble)) louis.nextStage();
        }
        else if(louis.stage == A_Stage.release_2.ordinal()){
            if(louis.stage_timer.seconds() >= 2.5) louis.nextStage();
            if(louis.stage_timer.seconds() <= 0.5) louis.arm_down();
            else {
                louis.claw_release();
                louis.arm_up();
            }
        }
        else if (louis.stage == A_Stage.End.ordinal()) {
            louis.stop_all();
        }
    }

    void B_state_machine(){
        if (louis.stage == B_Stage.Start.ordinal()) {
            louis.nextStage();
        }
        else if (louis.stage == B_Stage.put_fist_wobble.ordinal()) {
            if (louis.moveTo(B_put_first_wobble)) louis.nextStage();
        }
        else if(louis.stage == B_Stage.release_1.ordinal()){
            if(louis.stage_timer.seconds() >= 0.6) {
                louis.claw_release();
                louis.nextStage();
            }
            louis.arm_down();
        }
        else if (louis.stage == B_Stage.move_to_shoot.ordinal()) {
            if (louis.moveTo(B_move_to_shoot)) louis.nextStage();
            louis.arm_down();
//            louis.shooter_shoot();
        }
        else if (louis.stage == B_Stage.shoot.ordinal()) {
            if (louis.shoot_3_ring()) louis.nextStage();
        }
        else if (louis.stage == B_Stage.grab_second_wobble.ordinal()) {
            if (louis.moveTo(B_grab_second_wobble)) louis.nextStage();
        }
        else if(louis.stage == B_Stage.grab.ordinal()){
            if(louis.stage_timer.seconds() >= 1.5) louis.nextStage();
            if(louis.stage_timer.seconds() >= 0.4) louis.arm_up();
            louis.claw_grab();
        }
        else if (louis.stage == B_Stage.put_second_wobble.ordinal()) {
            if (louis.moveTo(B_put_second_wobble)) louis.nextStage();
        }
        else if(louis.stage == B_Stage.release_2.ordinal()){
            if(louis.stage_timer.seconds() >= 2.5) louis.nextStage();
            if(louis.stage_timer.seconds() <= 0.5) louis.arm_down();
            else {
                louis.claw_release();
                louis.arm_up();
            }
        }
        else if (louis.stage == B_Stage.End.ordinal()) {
            louis.stop_all();
        }
    }

    void C_state_machine(){

        if (louis.stage == C_Stage.Start.ordinal()) {
            louis.nextStage();
        }
        else if (louis.stage == C_Stage.move_to_shoot.ordinal()) {
            if (louis.moveTo(C_move_to_shoot)) louis.nextStage();
//            louis.arm_down();
//            louis.shooter_shoot();
        }
        else if (louis.stage == C_Stage.shoot.ordinal()) {
            if (louis.shoot_3_ring()) louis.nextStage();
        }
        else if (louis.stage == C_Stage.put_fist_wobble.ordinal()) {
            if (louis.moveTo(C_put_first_wobble)) louis.nextStage();
            louis.arm_down();
        }
        else if(louis.stage == C_Stage.release_1.ordinal()){
            louis.claw_release();
            louis.nextStage();
        }
        else if (louis.stage == C_Stage.move_to_shoot_2.ordinal()) {
            if (louis.moveTo(C_move_to_shoot_2)) louis.nextStage();
            louis.suck_spin();
        }
        else if (louis.stage == C_Stage.shoot_2.ordinal()) {
            if (louis.shoot_3_ring()) louis.nextStage();
        }
        else if (louis.stage == C_Stage.grab_second_wobble.ordinal()) {
            if (louis.moveTo(C_grab_second_wobble)) louis.nextStage();
            louis.arm_down();
            louis.claw_release();
        }
        else if(louis.stage == C_Stage.grab.ordinal()){
            if(louis.stage_timer.seconds() >= 0.7) louis.nextStage();
            louis.claw_grab();
        }
        else if (louis.stage == C_Stage.put_second_wobble.ordinal()) {
            if (louis.moveTo(C_put_second_wobble)) louis.nextStage();
            louis.arm_up();
        }
        else if(louis.stage == C_Stage.release_2.ordinal()){
            if(louis.stage_timer.seconds() >= 1) louis.nextStage();
            if(louis.stage_timer.seconds() <= 0.5) louis.arm_down();
            else {
                louis.claw_release();
            }
        }
        else if (louis.stage == C_Stage.go_line.ordinal()) {
            if (louis.moveTo(C_go_line)) louis.nextStage();
            louis.arm_up();

        }
        else if (louis.stage == C_Stage.End.ordinal()) {
            louis.stop_all();
            louis.arm_up();
        }
    }

    void setup_trajectory(){
        /**
         * A
         */
        A_put_first_wobble.add(new Pose(20, -150, 0));

//        A_release_1.add(new Pose(0, 0, 60));
//        A_release_1.add(new Pose(0, 0, -60));

        A_move_to_shoot.add(new Pose(-65, 0, 0));
        A_move_to_shoot.add(new Pose(0, 0, 180));

        A_grab_second_wobble.add(new Pose(40, -92, 0));
//        A_grab_second_wobble.add(new Pose(30, 0, 0));

        A_put_second_wobble.add(new Pose(-60, 120, 0));
        A_put_second_wobble.add(new Pose(0, 0, -90));
        /**
         * B
         */
        B_put_first_wobble.add(new Pose(0, -220, 0));
        B_put_first_wobble.add(new Pose(-20, 0, 0));

//        A_release_1.add(new Pose(0, 0, 60));
//        A_release_1.add(new Pose(0, 0, -60));

        B_move_to_shoot.add(new Pose(-50, 70, 0));
        B_move_to_shoot.add(new Pose(0, 0, 180));


        B_grab_second_wobble.add(new Pose(13, 0, 0));
        B_grab_second_wobble.add(new Pose(0, -92, 0));

        B_put_second_wobble.add(new Pose(0, 160, 0));
        B_put_second_wobble.add(new Pose(-20, 0, 0));
        B_put_second_wobble.add(new Pose(0, 0, -135));

        /**
         * C
         */


        C_move_to_shoot.add(new Pose(0, -160, 0));
        C_move_to_shoot.add(new Pose(0, 0, -20));

        C_put_first_wobble.add(new Pose(0, 0, 20));
        C_put_first_wobble.add(new Pose(0, -130, 0));
        C_put_first_wobble.add(new Pose(0, 0, 45));

        C_move_to_shoot_2.add(new Pose(0, 70 ,0));
        C_move_to_shoot_2.add(new Pose(0, 0 ,-45));
        C_move_to_shoot_2.add(new Pose(0, 150 ,0));

        C_grab_second_wobble.add(new Pose(0, 0, 180));
        C_grab_second_wobble.add(new Pose(40, -35, 0));

        C_put_second_wobble.add(new Pose(-80, 200, 0));
        C_put_second_wobble.add(new Pose(0, 0, -135));

        C_go_line.add(new Pose(0, 60, 0));
    }
}