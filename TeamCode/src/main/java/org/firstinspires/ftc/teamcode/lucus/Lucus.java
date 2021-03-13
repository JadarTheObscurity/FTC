package org.firstinspires.ftc.teamcode.lucus;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.JadarControl;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;

import java.util.ArrayList;
import java.util.List;

public class Lucus {
    public MecanumDriveTrain driveTrain;
    JadarControl control;
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    List<LynxModule> allHubs;
    DcMotorEx f_suck, b_suck, shoot, arm;
    DcMotorEx m_ly, m_ry, m_x;

    Servo claw_servo, push_servo;
    ElapsedTime shoot_timer = new ElapsedTime();

    Pose2d curr_pos = new Pose2d(0,0,0),
            last_pos = new Pose2d(0, 0, 0);


    public Lucus(HardwareMap hardwareMap){
        driveTrain = new MecanumDriveTrain(hardwareMap, true);
        control = new JadarControl(driveTrain);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        allHubs = hardwareMap.getAll(LynxModule.class);
        f_suck = hardwareMap.get(DcMotorEx.class, "fs");
        b_suck = hardwareMap.get(DcMotorEx.class, "bs");
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");
        m_ly = hardwareMap.get(DcMotorEx.class, "lf");
        m_ry = hardwareMap.get(DcMotorEx.class, "lb");
        m_x = hardwareMap.get(DcMotorEx.class, "rf");

        m_ly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_ly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_ry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_ry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        push_servo = hardwareMap.get(Servo.class, "push");
        claw_servo = hardwareMap.get(Servo.class, "claw");

        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(LynxModule module : allHubs) module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        webcam = new Webcam(hardwareMap, new Point(640, 480));
    }

    //State machine
    ElapsedTime stage_timer = new ElapsedTime();
    ElapsedTime sub_stage_timer = new ElapsedTime();
    public int stage = 0;
    public int sub_stage = 0;
    public void nextStage(){
        stage_timer.reset();
        sub_stage_timer.reset();
        reset_all();
        sub_stage = 0;
        stage++;
    }
    public void nextSubStage(){
        sub_stage_timer.reset();
        reset_all();
        sub_stage++;
    }

    //Drive Train Behavior
    int pose_index = 0;
    public boolean moveTo(Pose2d target){return control.moveTo_OW(last_pos, target, curr_pos );}
    public boolean moveTo(ArrayList<Pose2d> ps){
        if(pose_index >= ps.size()) return true;
        if(moveTo(ps.get(pose_index))) {
            reset_pos();
            pose_index++;
        }
        return false;
    }
    public void reset_pos(){
        stop_all();
        control.reset();
        last_pos.set(curr_pos);
    }

    public void reset_all(){
        reset_pos();
        pose_index = 0;
        shoot_timer.reset();
        shoot_num = 0;

    }

    public void update(){
        for(LynxModule module : allHubs) module.clearBulkCache();
        computeCoordinate();

    }


    double heading = 0;
    double last_heading = 0;
    double x_cm = 0;
    double y_cm = 0;
    double x_cm_raw = 0;
    double y_cm_raw = 0;

    double last_x_cm_raw = 0;
    double last_y_cm_raw = 0;
    static double pi = 3.14159;
    static double cmTOInch = 0.393701;
    /**
     *Dead wheel
     *all measured in cm
     */
    static double dead_wheel_diameter = 5.7;
    static double lateral_distance = 37;
    static double forward_offset = 6;
    static double encoder_cpr = 1460;
    static double cpr_to_rad = 2 * pi / encoder_cpr;
    static double distance_ratio = 0.0128;//dead_wheel_diameter/ 2 * cpr_to_rad;
    static double spin_ratio = dead_wheel_diameter / lateral_distance * cpr_to_rad * 1.01;
    static double x_correction_ratio = dead_wheel_diameter / 2 / forward_offset * cpr_to_rad;
    public void computeCoordinate(){

        heading =spin_ratio * (m_ly.getCurrentPosition() + m_ry.getCurrentPosition()) / 2;
        double det_heading = heading - last_heading;
        last_heading = heading;
        x_cm_raw = -(m_x.getCurrentPosition() * distance_ratio);
        y_cm_raw = (double)(m_ry.getCurrentPosition() - m_ly.getCurrentPosition()) / 2 * distance_ratio;
        double det_x = x_cm_raw - last_x_cm_raw + det_heading * x_correction_ratio;
        double det_y = y_cm_raw - last_y_cm_raw ;

        x_cm += Math.cos(heading) * det_x - Math.sin(heading) * det_y;
        y_cm += Math.sin(heading) * det_x + Math.cos(heading) * det_y;
        last_x_cm_raw = x_cm_raw;
        last_y_cm_raw = y_cm_raw;
        curr_pos.set(x_cm, y_cm, heading);
    }

    public void suck_spin(){
        f_suck.setPower(-1);
        b_suck.setPower(-0.7);
    }

    public void suck_reverse(){
        f_suck.setPower(1);
        b_suck.setPower(0.75);
    }

    public void suck_stop(){
        f_suck.setPower(0);
        b_suck.setPower(0);
    }

    public void arm_up(){
        arm.setTargetPosition(0);
        arm.setPower(0.5);
    }
    public void arm_down(){
        arm.setTargetPosition(1200);
        arm.setPower(0.4);
    }
    public void arm_stop(){
        arm.setPower(0);
    }

    public void claw_grab(){
        claw_servo.setPosition(0.5);}
    public void claw_release(){
        claw_servo.setPosition(0.8);}

    public void stop_all(){
        driveTrain.move(0, 0, 0);
        arm_stop();
        shooter_stop();
        suck_stop();
    }

    public void move(double x, double y, double r){
        driveTrain.move(x, y, r);
    }


    // intake

    //shooter

    public void shooter_shoot(){shoot.setVelocity(-2200);}
    public void shooter_stop(){shoot.setPower(0);}

    public void fire_on(){push_servo.setPosition(0.8);}
    public void fire_off(){push_servo.setPosition(0.3);}

    int shoot_num = 0;
    public boolean shoot_3_ring(){
        shooter_shoot();
        double sec_bias = 0;
        if(shoot_num == 0) sec_bias = 0.5;
        if(sec_bias <= shoot_timer.seconds() &&shoot_timer.seconds() < 0.3 + sec_bias)  fire_on();
        else if(shoot_timer.seconds() < 0.6 + sec_bias) fire_off();
        else {
            shoot_timer.reset();
            shoot_num++;
        }
        return shoot_num >= 3;
    }

}
