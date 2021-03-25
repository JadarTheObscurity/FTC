package org.firstinspires.ftc.teamcode.johanson;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.ArrayList;


@Autonomous(group = "Johanson")
public class JohansonAuto extends OpMode {
    Johanson johanson;
    @Override
    public void init() {
        johanson = new Johanson(hardwareMap);
        setup_trajectory();
        johanson.claw_grab();
    }

    @Override
    public void start() {
        super.start();
        zone = TargetZone.A;
    }

    @Override
    public void loop() {

        switch(zone){
            case A:
                A_state_machine();
                break;
            case B:
                B_state_machine();
                break;
            case C:
                C_state_machine();
                break;
        }
        johanson.update();
    }

    @Override
    public void stop() {
        johanson.finish();
    }

    enum TargetZone{
        A, B, C
    }

    TargetZone zone = TargetZone.A;

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
        go_line,
        End
    }
    enum B_Stage {
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

    ArrayList<Pose2d> A_move_to_shoot = new ArrayList<>();
    ArrayList<Pose2d> A_put_first_wobble = new ArrayList<>();
    ArrayList<Pose2d> A_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> A_put_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> A_go_line = new ArrayList<>();

    ArrayList<Pose2d> B_move_to_shoot = new ArrayList<>();
    ArrayList<Pose2d> B_put_first_wobble = new ArrayList<>();
    ArrayList<Pose2d> B_move_to_shoot_2 = new ArrayList<>();
    ArrayList<Pose2d> B_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> B_put_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> B_go_line = new ArrayList<>();

    ArrayList<Pose2d> C_move_to_shoot = new ArrayList<>();
    ArrayList<Pose2d> C_put_first_wobble = new ArrayList<>();
    ArrayList<Pose2d> C_move_to_shoot_2 = new ArrayList<>();
    ArrayList<Pose2d> C_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> C_put_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> C_go_line = new ArrayList<>();

    void A_state_machine(){
        if (johanson.stage == A_Stage.Start.ordinal()) {
            johanson.nextStage();
        }
        /**
         * Shoot
         */
        else if (johanson.stage == A_Stage.move_to_shoot.ordinal()) {
            if (johanson.moveTo(A_move_to_shoot)) johanson.nextStage();
        }
        else if (johanson.stage == A_Stage.shoot.ordinal()) {
            if (johanson.shoot_3_ring()) johanson.nextStage();
        }
        /**
         * First wobble
         */
        else if (johanson.stage == A_Stage.put_fist_wobble.ordinal()) {
            if (johanson.moveTo(A_put_first_wobble)) johanson.nextStage();
            johanson.arm_down();
        }
        else if(johanson.stage == A_Stage.release_1.ordinal()){
            johanson.claw_release();
            johanson.nextStage();
        }

        /**
         * Wobble 2
         */
        else if (johanson.stage == A_Stage.grab_second_wobble.ordinal()) {
            if (johanson.moveTo(A_grab_second_wobble)) johanson.nextStage();
            johanson.arm_down();
            johanson.claw_release();
        }
        else if(johanson.stage == A_Stage.grab.ordinal()){
            if(johanson.stage_timer.seconds() >= 0.7) johanson.nextStage();
            johanson.claw_grab();
        }
        else if (johanson.stage == A_Stage.put_second_wobble.ordinal()) {
            if (johanson.moveTo(A_put_second_wobble)) johanson.nextStage();
            johanson.arm_up();
        }
        else if(johanson.stage == A_Stage.release_2.ordinal()){
            if(johanson.stage_timer.seconds() >= 1) johanson.nextStage();
            johanson.arm_down();
            johanson.claw_release();
        }
        /**
         * Line
         */
        else if (johanson.stage == A_Stage.go_line.ordinal()) {
            if (johanson.moveTo(A_go_line)) johanson.nextStage();
            johanson.arm_up();
        }
        else if (johanson.stage == A_Stage.End.ordinal()) {
            johanson.stop_all();
            johanson.arm_up();
        }
    }

    void B_state_machine(){
        if (johanson.stage == B_Stage.Start.ordinal()) {
            johanson.nextStage();
        }

        /**
         * Shoot
         */
        else if (johanson.stage == B_Stage.move_to_shoot.ordinal()) {
            if (johanson.moveTo(B_move_to_shoot)) johanson.nextStage();
        }
        else if (johanson.stage == B_Stage.shoot.ordinal()) {
            if (johanson.shoot_3_ring()) johanson.nextStage();
        }
        /**
         * First wobble
         */
        else if (johanson.stage == B_Stage.put_fist_wobble.ordinal()) {
            if (johanson.moveTo(B_put_first_wobble)) johanson.nextStage();
            johanson.arm_down();
        }
        else if(johanson.stage == B_Stage.release_1.ordinal()){
            johanson.claw_release();
            johanson.nextStage();
        }
        /**
         * Shoot 2
         */
        else if (johanson.stage == B_Stage.move_to_shoot_2.ordinal()) {
            if (johanson.moveTo(B_move_to_shoot_2)) johanson.nextStage();
            if(johanson.pose_index <= 1);
            else if(johanson.pose_index == 2){
                johanson.control.time_mult = 3;
            }
            else johanson.control.time_mult = 1.3;
            johanson.suck_spin();
        }
        else if (johanson.stage == B_Stage.shoot_2.ordinal()) {
            if (johanson.shoot_3_ring()) johanson.nextStage();
        }
        /**
         * Wobble 2
         */
        else if (johanson.stage == B_Stage.grab_second_wobble.ordinal()) {
            if (johanson.moveTo(B_grab_second_wobble)) johanson.nextStage();
            johanson.arm_down();
            johanson.claw_release();
        }
        else if(johanson.stage == B_Stage.grab.ordinal()){
            if(johanson.stage_timer.seconds() >= 0.7) johanson.nextStage();
            johanson.claw_grab();
        }
        else if (johanson.stage == B_Stage.put_second_wobble.ordinal()) {
            if (johanson.moveTo(B_put_second_wobble)) johanson.nextStage();
            johanson.arm_up();
        }
        else if(johanson.stage == B_Stage.release_2.ordinal()){
            if(johanson.stage_timer.seconds() >= 1) johanson.nextStage();
            johanson.arm_down();
            johanson.claw_release();
        }
        /**
         * Line
         */
        else if (johanson.stage == B_Stage.go_line.ordinal()) {
            if (johanson.moveTo(B_go_line)) johanson.nextStage();
            johanson.arm_up();
        }
        else if (johanson.stage == B_Stage.End.ordinal()) {
            johanson.stop_all();
            johanson.arm_up();
        }
    }

    void C_state_machine(){
        if (johanson.stage == C_Stage.Start.ordinal()) {
            johanson.nextStage();
        }

        /**
         * Shoot
         */
        else if (johanson.stage == C_Stage.move_to_shoot.ordinal()) {
            if (johanson.moveTo(C_move_to_shoot)) johanson.nextStage();
        }
        else if (johanson.stage == C_Stage.shoot.ordinal()) {
            if (johanson.shoot_3_ring()) johanson.nextStage();
        }
        /**
         * First wobble
         */
        else if (johanson.stage == C_Stage.put_fist_wobble.ordinal()) {
            if (johanson.moveTo(C_put_first_wobble)) johanson.nextStage();
            johanson.arm_down();
        }
        else if(johanson.stage == C_Stage.release_1.ordinal()){
            johanson.claw_release();
            johanson.nextStage();
        }
        /**
         * Shoot 2
         */
        else if (johanson.stage == C_Stage.move_to_shoot_2.ordinal()) {
            if (johanson.moveTo(C_move_to_shoot_2)) johanson.nextStage();
            if(johanson.pose_index <= 1);
            else if(johanson.pose_index == 2){
                johanson.control.time_mult = 3;
            }
            else johanson.control.time_mult = 1.3;
            johanson.suck_spin();
        }
        else if (johanson.stage == C_Stage.shoot_2.ordinal()) {
            if (johanson.shoot_3_ring()) johanson.nextStage();
        }
        /**
         * Wobble 2
         */
        else if (johanson.stage == C_Stage.grab_second_wobble.ordinal()) {
            if (johanson.moveTo(C_grab_second_wobble)) johanson.nextStage();
            johanson.arm_down();
            johanson.claw_release();
        }
        else if(johanson.stage == C_Stage.grab.ordinal()){
            if(johanson.stage_timer.seconds() >= 0.7) johanson.nextStage();
            johanson.claw_grab();
        }
        else if (johanson.stage == C_Stage.put_second_wobble.ordinal()) {
            if (johanson.moveTo(C_put_second_wobble)) johanson.nextStage();
            johanson.arm_up();
        }
        else if(johanson.stage == C_Stage.release_2.ordinal()){
            if(johanson.stage_timer.seconds() >= 1) johanson.nextStage();
            johanson.arm_down();
            johanson.claw_release();
        }
        /**
         * Line
         */
        else if (johanson.stage == C_Stage.go_line.ordinal()) {
            if (johanson.moveTo(C_go_line)) johanson.nextStage();
            johanson.arm_up();
        }
        else if (johanson.stage == C_Stage.End.ordinal()) {
            johanson.stop_all();
            johanson.arm_up();
        }
    }

    Pose2d start_pos = new Pose2d(-150, -160, 0);
    Pose2d shoot_1_pos = new Pose2d(-120, -10, Math.toRadians(-20));
    Pose2d second_wobble_pos = new Pose2d(-75, -94, 0);
    Pose2d shoot_2_pos = new Pose2d(-93, -20, Math.toRadians(-10));
    Pose2d end_pos = new Pose2d(-150, -160, 0);

    void setup_trajectory(){

        johanson.setStartPose(start_pos);


        /**
         * A
         */
        A_move_to_shoot.add(shoot_1_pos);

        A_put_first_wobble.add(new Pose2d(-120, 20, Math.toRadians(-150)));

        A_grab_second_wobble.add(second_wobble_pos);

        A_put_second_wobble.add(new Pose2d(-120, 10, Math.toRadians(-140)));

        A_go_line.add(end_pos);

        /**
         * B
         */
        B_move_to_shoot.add(new Pose2d(-130, -10, Math.toRadians(160)));

        B_put_first_wobble.add(new Pose2d(-90, 80, Math.toRadians(150)));

        B_move_to_shoot_2.add(new Pose2d(-96, 0, Math.toRadians(180)));
        B_move_to_shoot_2.add(new Pose2d(-96, -20, Math.toRadians(180)));
        B_move_to_shoot_2.add(new Pose2d(-93, -72, Math.toRadians(180)));
        B_move_to_shoot_2.add(shoot_2_pos);

        B_grab_second_wobble.add(new Pose2d(-53, -94, Math.toRadians(0)));

        B_put_second_wobble.add(new Pose2d(-90, 60, Math.toRadians(-140)));

        B_go_line.add(end_pos);

        /**
         * C
         */

        C_move_to_shoot.add(shoot_1_pos);

        C_put_first_wobble.add(new Pose2d(-120, 120, Math.toRadians(230)));


        C_move_to_shoot_2.add(new Pose2d(-96, 0 ,Math.toRadians(180)));
        C_move_to_shoot_2.add(new Pose2d(-96, -20 ,Math.toRadians(180)));
        C_move_to_shoot_2.add(new Pose2d(-93, -72 ,Math.toRadians(180)));
        C_move_to_shoot_2.add(shoot_2_pos);

        C_grab_second_wobble.add(new Pose2d(-50, -20, Math.toRadians(0)));
        C_grab_second_wobble.add(new Pose2d(-53, -94, Math.toRadians(0)));

        C_put_second_wobble.add(new Pose2d(-50, 0, Math.toRadians(-140)));
        C_put_second_wobble.add(new Pose2d(-130, 100, Math.toRadians(-140)));

        C_go_line.add(end_pos);
    }
}
