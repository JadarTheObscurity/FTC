package org.firstinspires.ftc.teamcode.louis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.JadarControl;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

public class Louis {
    public MecanumDriveTrain driveTrain;
    JadarControl control;
    OpenCvCamera webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    List<LynxModule> allHubs;
    DcMotorEx f_suck, b_suck, shoot, arm;
    Servo claw_servo, push_servo;
    ElapsedTime shoot_timer = new ElapsedTime();
    FtcDashboard dashboard;


    public Louis(HardwareMap hardwareMap){
        driveTrain = new MecanumDriveTrain(hardwareMap, false);
        control = new JadarControl(driveTrain);
        allHubs = hardwareMap.getAll(LynxModule.class);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        f_suck = hardwareMap.get(DcMotorEx.class, "fs");
        b_suck = hardwareMap.get(DcMotorEx.class, "bs");
        shoot = hardwareMap.get(DcMotorEx.class, "shoot");


        push_servo = hardwareMap.get(Servo.class, "push");
        claw_servo = hardwareMap.get(Servo.class, "claw");

        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(LynxModule module : allHubs) module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


        dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        dashboard.startCameraStream(webcam, 10);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
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
    public boolean moveTo(double x, double y, double r){
        return control.moveTo_Encoder(x, y, r);
    }
    public boolean moveTo(Pose p){return moveTo(p.x, p.y, p.r);}
    public boolean moveTo(ArrayList<Pose> ps){
        if(pose_index >= ps.size()) return true;
        if(moveTo(ps.get(pose_index))) {
            reset_pos();
            pose_index++;
        }
        return false;
    }
    public void reset_pos(){
        stop_all();
        driveTrain.reset_pos();
        control.reset();
    }

    public void reset_all(){
        reset_pos();
        pose_index = 0;
        shoot_timer.reset();
        shoot_num = 0;

    }

    public void update(){
//        for(LynxModule module : allHubs) module.clearBulkCache();
    }


    public void suck_spin(){
        f_suck.setPower(-0.4);
        b_suck.setPower(-0.7);
    }

    public void suck_reverse(){
        f_suck.setPower(0.3);
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

    public void shooter_shoot(){shoot.setVelocity(-2300);}
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

    public boolean see_tower(){
        return pipeline.found;
    }

    public double tower_x(){
        return pipeline.x - 240;
    }

}
