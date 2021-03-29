package org.firstinspires.ftc.teamcode.johanson;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
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
import org.firstinspires.ftc.teamcode.util.WayPoint;
import org.firstinspires.ftc.teamcode.util.Webcam;

import java.util.ArrayList;
import java.util.List;

public class Johanson{

    public MecanumDriveTrain driveTrain;
    JadarControl control;
    Webcam webcam;
    TowerPipeline pipeline = new TowerPipeline(TowerPipeline.Tower.Blue);
    static List<LynxModule> allHubs;
    static DcMotorEx suck_motor, shoot_motor_1, shoot_motor_2,arm_motor;
    static DcMotorEx m_ly, m_ry, m_x;

    Servo claw_servo, push_servo, load_servo_l, load_servo_r;
    protected ElapsedTime shoot_timer = new ElapsedTime();

    static Pose2d curr_pos = new Pose2d(0,0,0),
            last_pos = new Pose2d(0, 0, 0),
            start_pos = new Pose2d(0, 0, 0);

    protected static FtcDashboard dashboard;
    static Canvas fieldOverlay;
    public static TelemetryPacket packet = new TelemetryPacket();

    static Thread ow_thread = new OdometryWheelThread();


    final double shootPower = 1;
    final double suckPower = -0.7;
    final double reversesuckPower = 1;
    final double plateState_collect = 0.14;
    final double plateState_lift = 0.33;
    final double pushState_wait = 0.1;
    final double pushState_push = 0.55;
    final double armState_up = 0;
    final int armState_down = 1600;
    final double clawState_loose=1; //0.8
    final double clawState_catch=0.5;//0.3
    final double shoot_duration = 0.6;

    public Johanson(HardwareMap hardwareMap) {
        setUpHardware(hardwareMap);
        heading=x_cm=y_cm=0;
        ow_thread.start();
        setStartPose(new Pose2d(0, 0, 0));
        packet = new TelemetryPacket();
    }

    void setUpHardware(HardwareMap hardwareMap){
        driveTrain = new MecanumDriveTrain(hardwareMap, false);
        control = new JadarControl(driveTrain);
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        allHubs = hardwareMap.getAll(LynxModule.class);
        suck_motor = hardwareMap.get(DcMotorEx.class, "suck");
        shoot_motor_1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shoot_motor_2 = hardwareMap.get(DcMotorEx.class, "shoot2");
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
        load_servo_l = hardwareMap.get(Servo.class, "plate1");
        load_servo_r = hardwareMap.get(Servo.class, "plate2");
        load_servo_r.setDirection(Servo.Direction.REVERSE);

//        shoot_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shoot_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(LynxModule module : allHubs) module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        webcam = new Webcam(hardwareMap, new Point(640, 480));
    }

    void drawRobot(){
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();
        Pose2d currentPose = new Pose2d(curr_pos.getY() * cmTOInch,-curr_pos.getX() * cmTOInch, -curr_pos.getR()+Math.toRadians(90));
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);
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
    public boolean moveTo(Pose2d target){return control.moveTo(last_pos, target, curr_pos );}
    public boolean moveTo(WayPoint target){return control.moveTo(last_pos, target, curr_pos );}
    public boolean moveTo(ArrayList<Pose2d> ps){
        if(pose_index >= ps.size()) return true;
        if(moveTo(ps.get(pose_index))) {
            reset_pos();
            pose_index++;
        }
        return false;
    }

    public void finish_moveTo(){
        control.reset();
        last_pos.set(curr_pos);
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
        finish_moveTo();
    }

    public void reset_all(){
        reset_pos();
        pose_index = 0;
        shoot_timer.reset();
        shoot_num = 0;
        control.time_mult = 1;

    }

    public void update(){
        drawRobot();
        packet.put("pose", curr_pos.toString());
        dashboard.sendTelemetryPacket(packet);
    }

    public void finish(){
        //Stop the dead wheel thread
        ow_thread.interrupt();
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
    public static double x_correction_ratio = 0.088;//dead_wheel_diameter / 2 / forward_offset * cpr_to_rad;
    public static void computeCoordinate(){

        heading =spin_ratio * (m_ly.getCurrentPosition() + m_ry.getCurrentPosition()) / 2 + start_pos.getR();
        double det_heading = heading - last_heading;
        last_heading = heading;
        x_cm_raw = -(m_x.getCurrentPosition() * distance_ratio);
        y_cm_raw = (double)(m_ry.getCurrentPosition() - m_ly.getCurrentPosition()) / 2 * distance_ratio;
        double det_x = x_cm_raw - last_x_cm_raw + det_heading * x_correction_ratio;
        double det_y = y_cm_raw - last_y_cm_raw ;

        //TODO Add @det_heading to below to see whether it will be better
        x_cm += Math.cos(heading - det_heading / 2) * det_x - Math.sin(heading - det_heading / 2) * det_y;
        y_cm += Math.sin(heading - det_heading / 2) * det_x + Math.cos(heading - det_heading / 2) * det_y;
        last_x_cm_raw = x_cm_raw;
        last_y_cm_raw = y_cm_raw;
        curr_pos.set(x_cm + start_pos.getX(), y_cm + start_pos.getY(), heading);
    }


    private static class OdometryWheelThread extends Thread{
    public void run(){
        try
        {
            while (!isInterrupted())
            {
                for(LynxModule module : allHubs) module.clearBulkCache();
                computeCoordinate();
                Thread.sleep(10);
            }
        }
        catch (InterruptedException e) {System.out.println(e.toString());}

    }
}

//==============================================================================================================================

    public void arm_up(){
        arm_motor.setTargetPosition(0);
        arm_motor.setPower(0.5);
    }
    public void arm_down(){
        arm_motor.setTargetPosition(armState_down);
        arm_motor.setPower(0.4);
    }
    public void arm_stop(){
        arm_motor.setPower(0);
    }

    public void claw_grab(){
        claw_servo.setPosition(clawState_catch);}
    public void claw_release(){
        claw_servo.setPosition(clawState_loose);}

    public void stop_all(){
        driveTrain.move(0, 0, 0);
        if(!arm_motor.isBusy()) arm_stop();
        shooter_stop();
        suck_stop();
    }

    public void move(double x, double y, double r){
        driveTrain.move(x, y, r);
    }


    // intake
    public void suck_spin(){
        suck_motor.setPower(suckPower);
    }

    public void suck_reverse(){
        suck_motor.setPower(reversesuckPower);
    }

    public void suck_stop(){
        suck_motor.setPower(0);
    }
    //shooter

    public void shooter_shoot(){
        shoot_motor_1.setPower(shootPower);
        shoot_motor_2.setPower(shootPower);
    }
    public void shooter_stop(){
        shoot_motor_1.setPower(0);
        shoot_motor_2.setPower(0);
    }

    public void load_up(){
        load_servo_l.setPosition(plateState_lift);
        load_servo_r.setPosition(plateState_lift);
    }
    public void load_down(){
        load_servo_l.setPosition(plateState_collect);
        load_servo_r.setPosition(plateState_collect);
    }

    public void fire_on(){push_servo.setPosition(pushState_push);}
    public void fire_off(){push_servo.setPosition(pushState_wait);}

    int shoot_num = 0;
    public boolean shoot_3_ring(){
        shooter_shoot();
        load_up();
        double sec_bias = 0;
        if(shoot_num == 0) sec_bias = 0.5;
        if(sec_bias <= shoot_timer.seconds() &&shoot_timer.seconds() < shoot_duration / 2 + sec_bias)  fire_on();
        else if(shoot_timer.seconds() < shoot_duration+ sec_bias) fire_off();
        else {
            shoot_timer.reset();
            shoot_num++;
        }
        return shoot_num >= 3;
    }

}
