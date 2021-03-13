package org.firstinspires.ftc.teamcode.lucus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.JadarControl;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.ArrayList;

@Autonomous(name = "Lucus Auto 1", group = "Lucus")
public class LucusAuto1 extends OpMode {
    Lucus lucus;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    DcMotorEx arm;
    Servo claw;

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
        packet.put("x", lucus.driveTrain.get_x());
        packet.put("y", lucus.driveTrain.get_y());
        packet.put("target x", lucus.control.x_target);
        packet.put("target y", lucus.control.y_target);
        packet.put("error y", lucus.control.y_error);
        packet.put("error y * kp", lucus.control.y_error * JadarControl.y_kp);
        dashboard.sendTelemetryPacket(packet);

        C_state_machine();
        lucus.update();
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

        /**
         * C
         */

        C_move_to_shoot.add(new Pose2d(0, -50, 0));
        C_move_to_shoot.add(new Pose2d(0, 0, 0));
//        C_move_to_shoot.add(new Pose2d(30, 0, 0));
//        C_move_to_shoot.add(new Pose2d(0, 0, 0));
//        C_move_to_shoot.add(new Pose(0, -160, 0));
//        C_move_to_shoot.add(new Pose(0, 0, -20));
//
//        C_put_first_wobble.add(new Pose(0, 0, 20));
//        C_put_first_wobble.add(new Pose(0, -130, 0));
//        C_put_first_wobble.add(new Pose(0, 0, 45));
//
//        C_move_to_shoot_2.add(new Pose(0, 70 ,0));
//        C_move_to_shoot_2.add(new Pose(0, 0 ,-45));
//        C_move_to_shoot_2.add(new Pose(0, 150 ,0));
//
//        C_grab_second_wobble.add(new Pose(0, 0, 180));
//        C_grab_second_wobble.add(new Pose(40, -35, 0));
//
//        C_put_second_wobble.add(new Pose(-80, 200, 0));
//        C_put_second_wobble.add(new Pose(0, 0, -135));
//
//        C_go_line.add(new Pose(0, 60, 0));
    }
}