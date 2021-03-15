package org.firstinspires.ftc.teamcode.lucus;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.JadarControl;
import org.firstinspires.ftc.teamcode.util.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;

import java.util.ArrayList;
import java.util.List;

@Config
public class Lucus {
    public MecanumDriveTrain driveTrain;
    JadarControl control;
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    static List<LynxModule> allHubs;
    static DcMotorEx f_suck, b_suck, shoot, arm;
    static DcMotorEx m_ly, m_ry, m_x;

    Servo claw_servo, push_servo;
    ElapsedTime shoot_timer = new ElapsedTime();

    static  Pose2d curr_pos = new Pose2d(0,0,0),
            last_pos = new Pose2d(0, 0, 0),
            start_pos = new Pose2d(0, 0, 0);

    static FtcDashboard dashboard;
    static Canvas fieldOverlay;
    public static TelemetryPacket packet = new TelemetryPacket();


    public Lucus(HardwareMap hardwareMap){
        driveTrain = new MecanumDriveTrain(hardwareMap, false);
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

        dashboard = FtcDashboard.getInstance();
        fieldOverlay = packet.fieldOverlay();

        push_servo = hardwareMap.get(Servo.class, "push");
        claw_servo = hardwareMap.get(Servo.class, "claw");

        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(LynxModule module : allHubs) module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        webcam = new Webcam(hardwareMap, new Point(640, 480));

        heading=x_cm=y_cm=0;

        setStartPose(new Pose2d(0, 0, 0));
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
    public int pose_index = 0;
    public boolean moveTo(Pose2d target){return control.moveTo_OW(last_pos, target, curr_pos );}
    public boolean moveTo(ArrayList<Pose2d> ps){
        if(pose_index >= ps.size()) return true;
        if(moveTo(ps.get(pose_index))) {
            reset_pos();
            pose_index++;
        }
        return false;
    }

    public void setStartPose(Pose2d pose){
        last_pos.set(pose);
        start_pos.set(pose);
        curr_pos.set(pose);

        heading = pose.getR();
        last_heading = heading;
        x_cm = 0;
        y_cm = 0;
        last_x_cm_raw = 0;
        last_y_cm_raw = 0;
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
        control.time_mult = 1;

    }

    public void update(){
        for(LynxModule module : allHubs) module.clearBulkCache();
        computeCoordinate();

    }

    public void finish(){

    }


    static double heading = 0;
    static double last_heading = 0;
    static double x_cm = 0;
    static double y_cm = 0;
    static double x_cm_raw = 0;
    static double y_cm_raw = 0;

    static double last_x_cm_raw = 0;
    static double last_y_cm_raw = 0;
    static double pi = 3.14159;
    static double cmTOInch = 0.393701;
    /**
     *Dead wheel
     *all measured in cm
     */
    static double dead_wheel_diameter = 5.7;
    static double lateral_distance = 37;
    static double forward_offset = 0.1;//6
    static double encoder_cpr = 1460;
    static double cpr_to_rad = 2 * pi / encoder_cpr;
    static double distance_ratio = 0.0128;//dead_wheel_diameter/ 2 * cpr_to_rad;
    static double spin_ratio = dead_wheel_diameter / lateral_distance * cpr_to_rad * 1.0322;
    public static double x_correction_ratio = 1.75;//dead_wheel_diameter / 2 / forward_offset * cpr_to_rad;
    public void computeCoordinate(){

        heading =spin_ratio * (m_ly.getCurrentPosition() + m_ry.getCurrentPosition()) / 2 + start_pos.getR();
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
        curr_pos.set(x_cm + start_pos.getX(), y_cm + start_pos.getY(), heading);

        packet = new TelemetryPacket();
        packet.put("x_raw", x_cm_raw);
        fieldOverlay = packet.fieldOverlay();
        Pose2d currentPose = new Pose2d(curr_pos.getY() * cmTOInch,-curr_pos.getX() * cmTOInch, -curr_pos.getR()+Math.toRadians(90));
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);
        dashboard.sendTelemetryPacket(packet);
    }

        /*
    private static class OdometryWheel extends Thread{
        double pi = 3.1415;
        double dead_wheel_diameter = 5.7;
        double lateral_distance = 37;
        double forward_offset = 6;
        double encoder_cpr = 1460;
        double cpr_to_rad = 2 * pi / encoder_cpr;
        double distance_ratio = 0.0128;//dead_wheel_diameter/ 2 * cpr_to_rad;
        double spin_ratio = dead_wheel_diameter / lateral_distance * cpr_to_rad * 1.032;
        double x_correction_ratio = dead_wheel_diameter / 2 / forward_offset * cpr_to_rad;
        public void run(){
            try
            {
                while (!isInterrupted())
                {
                    for (LynxModule module : allHubs) {
                        module.clearBulkCache();
                    }
                    heading = spin_ratio * (m_ly.getCurrentPosition() + m_ry.getCurrentPosition()) / 2 + start_pos.getR();

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

                    curr_pos.set(x_cm + start_pos.getX(), y_cm + start_pos.getY(), heading);

                    packet = new TelemetryPacket();
                    packet.put("ly", m_ly.getCurrentPosition());
                    packet.put("ry", m_ry.getCurrentPosition());
                    packet.put("x", m_x.getCurrentPosition());
                    packet.put("angle", Math.toDegrees(heading));

                    packet.put("x_cm_raw", x_cm_raw);
                    packet.put("y_cm_raw", y_cm_raw);
                    packet.put("det_x", det_x);
                    packet.put("det_y", det_y);
                    packet.put("x_cm", x_cm);
                    packet.put("y_cm", y_cm);
                    packet.put("det_heading", det_heading);
                    packet.put("correction", det_heading * x_correction_ratio);
                    fieldOverlay = packet.fieldOverlay();
                    Pose2d currentPose = new Pose2d(curr_pos.getY() * cmTOInch,-curr_pos.getX() * cmTOInch, -curr_pos.getR()+Math.toRadians(90));
                    fieldOverlay.setStroke("#3F51B5");
                    DashboardUtil.drawRobot(fieldOverlay, currentPose);
                    dashboard.sendTelemetryPacket(packet);
                    Thread.sleep(10);
                }
            }
            catch (InterruptedException e) {System.out.println(e.toString());}

        }
    }

*/
    public void suck_spin(){
        f_suck.setPower(-0.5);
        b_suck.setPower(-0.9);
    }

    public void suck_reverse(){
        f_suck.setPower(0.15);
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
        claw_servo.setPosition(0.4);}
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
        if(sec_bias <= shoot_timer.seconds() &&shoot_timer.seconds() < 0.35 + sec_bias)  fire_on();
        else if(shoot_timer.seconds() < 0.7+ sec_bias) fire_off();
        else {
            shoot_timer.reset();
            shoot_num++;
        }
        return shoot_num >= 3;
    }

}
