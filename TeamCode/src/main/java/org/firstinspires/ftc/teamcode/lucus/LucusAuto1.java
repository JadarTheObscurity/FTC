package org.firstinspires.ftc.teamcode.lucus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.ArrayList;

@Autonomous(name = "Lucus Auto 1", group = "Lucus")
public class LucusAuto1 extends OpMode {
    Lucus lucus;
    FtcDashboard dashboard;
    DcMotorEx arm;
    Servo claw;
    Canvas fieldOverlay;
    public static double cmTOInch = 0.393701;


    @Override
    public void init() {
        lucus = new Lucus(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        telemetry.addData("stage", lucus.stage);
        telemetry.update();
        setup_trajectory();

        lucus.claw_grab();

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



    ArrayList<Pose2d> C_put_first_wobble = new ArrayList<>();
    ArrayList<Pose2d> C_release_1 = new ArrayList<>();
    ArrayList<Pose2d> C_move_to_shoot = new ArrayList<>();
    ArrayList<Pose2d> C_move_to_shoot_2 = new ArrayList<>();
    ArrayList<Pose2d> C_grab_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> C_put_second_wobble = new ArrayList<>();
    ArrayList<Pose2d> C_go_line = new ArrayList<>();

    @Override
    public void loop() {
        telemetry.addData("Curr Stage", lucus.stage);
        telemetry.update();


        C_state_machine();
        lucus.update();
    }

    @Override
    public void stop() {
        lucus.finish();
    }

    void C_state_machine(){

        if (lucus.stage == C_Stage.Start.ordinal()) {
            lucus.nextStage();
        }
        else if (lucus.stage == C_Stage.move_to_shoot.ordinal()) {
            if (lucus.moveTo(C_move_to_shoot)) lucus.nextStage();
//            louis.arm_down();
//            louis.shooter_shoot();
        }
        else if (lucus.stage == C_Stage.shoot.ordinal()) {
            if (lucus.shoot_3_ring()) lucus.nextStage();
        }
        else if (lucus.stage == C_Stage.put_fist_wobble.ordinal()) {
            if (lucus.moveTo(C_put_first_wobble)) lucus.nextStage();
            lucus.arm_down();
        }
        else if(lucus.stage == C_Stage.release_1.ordinal()){
            lucus.claw_release();
            lucus.nextStage();
        }
        else if (lucus.stage == C_Stage.move_to_shoot_2.ordinal()) {
            if (lucus.moveTo(C_move_to_shoot_2)) lucus.nextStage();
            if(lucus.pose_index <= 1);
            else if(lucus.pose_index == 2){
                lucus.control.time_mult = 3;

            }
            else lucus.control.time_mult = 1.3;
            lucus.suck_spin();
        }
        else if (lucus.stage == C_Stage.shoot_2.ordinal()) {
            if (lucus.shoot_3_ring()) lucus.nextStage();
        }
        else if (lucus.stage == C_Stage.grab_second_wobble.ordinal()) {
            if (lucus.moveTo(C_grab_second_wobble)) lucus.nextStage();
            lucus.arm_down();
            lucus.claw_release();
        }
        else if(lucus.stage == C_Stage.grab.ordinal()){
            if(lucus.stage_timer.seconds() >= 0.7) lucus.nextStage();
            lucus.claw_grab();
        }
        else if (lucus.stage == C_Stage.put_second_wobble.ordinal()) {
            if (lucus.moveTo(C_put_second_wobble)) lucus.nextStage();
            lucus.arm_up();
        }
        else if(lucus.stage == C_Stage.release_2.ordinal()){
            if(lucus.stage_timer.seconds() >= 1) lucus.nextStage();
            if(lucus.stage_timer.seconds() <= 0.5) lucus.arm_down();
            else {
                lucus.claw_release();
            }
        }
        else if (lucus.stage == C_Stage.go_line.ordinal()) {
            if (lucus.moveTo(C_go_line)) lucus.nextStage();
            lucus.arm_up();

        }
        else if (lucus.stage == C_Stage.End.ordinal()) {
            lucus.stop_all();
            lucus.arm_up();
        }
    }

    void setup_trajectory(){

        lucus.setStartPose(new Pose2d(-150, -150, Math.toRadians(180)));

        /**
         * C
         */

        C_move_to_shoot.add(new Pose2d(-130, -10, Math.toRadians(157)));

        C_put_first_wobble.add(new Pose2d(-120, 120, Math.toRadians(230)));


        C_move_to_shoot_2.add(new Pose2d(-96, 0 ,Math.toRadians(180)));
        C_move_to_shoot_2.add(new Pose2d(-96, -20 ,Math.toRadians(180)));
        C_move_to_shoot_2.add(new Pose2d(-93, -72 ,Math.toRadians(180)));
        C_move_to_shoot_2.add(new Pose2d(-93, -20 ,Math.toRadians(170)));

        C_grab_second_wobble.add(new Pose2d(-50, -20, Math.toRadians(0)));
        C_grab_second_wobble.add(new Pose2d(-53, -94, Math.toRadians(0)));

        C_put_second_wobble.add(new Pose2d(-50, 0, Math.toRadians(-140)));
        C_put_second_wobble.add(new Pose2d(-130, 100, Math.toRadians(-140)));

        C_go_line.add(new Pose2d(-130, -140, Math.toRadians(-180)));
    }
}