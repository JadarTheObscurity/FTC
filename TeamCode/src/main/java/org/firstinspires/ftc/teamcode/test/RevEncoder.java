package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.IMU;
import org.firstinspires.ftc.teamcode.util.MyMath;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
public class RevEncoder extends LinearOpMode {
    DcMotorEx m_ly, m_ry, m_x;
    DcMotorEx lf, lb, rf, rb;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        m_ly = hardwareMap.get(DcMotorEx.class, "lf");
        m_ry = hardwareMap.get(DcMotorEx.class, "lb");
        m_x = hardwareMap.get(DcMotorEx.class, "rf");
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dashboard = FtcDashboard.getInstance();
        imu = new IMU(hardwareMap);
        waitForStart();
        drive();
    }

    void drive(){
        while (opModeIsActive()) {
            move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            packet = new TelemetryPacket();
            computeCoordinae();
            drawRobot();
            packet.put("curr pos", curr_pos.toString());
            packet.put("ly", m_ly.getCurrentPosition());
            packet.put("ry", m_ry.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }
    }

    static double cmTOInch = 0.393701;
    Pose2d curr_pos = new Pose2d(0, 0, 0);
    static Canvas fieldOverlay;
    void drawRobot(){
        fieldOverlay = packet.fieldOverlay();
        Pose2d currentPose = new Pose2d(curr_pos.getY() * cmTOInch,-curr_pos.getX() * cmTOInch, -curr_pos.getR()+Math.toRadians(90));
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);
    }



    static double dead_wheel_diameter = 4.8;
    static double lateral_distance = 37;
    static double forward_offset = 13.64;
    static double encoder_cpr = 8192;
    static double cpr_to_rad = 2 * Math.PI / encoder_cpr;
    static double distance_ratio = dead_wheel_diameter / 2 * cpr_to_rad;
    static double spin_ratio = -0.9982e-4;//dead_wheel_diameter / lateral_distance * cpr_to_rad;
    public static double y_ratio = 312.0/308;
    public static double x_ratio = 163/153;
    public static double[] curr_deadwheel_encoder = {0, 0, 0};
    double ry_ratio = 1.035;

    public static double[] last_deadwheel_encoder = {0, 0, 0};

    double heading=0, last_heading=0, x_cm_raw=0, y_cm_raw=0, x_cm=0, y_cm=0;
    void calculate_spin_ratio(){
        int turn_count  = 0;
        while(opModeIsActive()){
            packet = new TelemetryPacket();
            double heading = imu.getHeading(AngleUnit.RADIANS);
            if(heading < Math.PI / 2){
                move(0, 0, -0.2);
            }
            else move(0, 0, 0);

            packet.put("heading", heading);
            packet.put("calculated spin ratio", heading / (m_ly.getCurrentPosition() + m_ry.getCurrentPosition() * ry_ratio) * 2);
            packet.put("origin ratio", spin_ratio);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    void calculate_lateral_distance(){
        while(opModeIsActive()){
            packet = new TelemetryPacket();
            double heading = (m_ly.getCurrentPosition() + m_ry.getCurrentPosition() * ry_ratio) / 2 * spin_ratio;
            if(heading < Math.PI / 2){
                move(0, 0, -0.2);
            }
            else move(0, 0, 0);

            packet.put("heading", heading);
            packet.put("calculated forward offset", (m_x.getCurrentPosition()) * distance_ratio / heading);
            packet.put("origin forward offset", forward_offset);
            dashboard.sendTelemetryPacket(packet);
        }
    }

    void computeCoordinae(){

        curr_deadwheel_encoder[0] = m_ly.getCurrentPosition();
        curr_deadwheel_encoder[1] = m_ry.getCurrentPosition() * ry_ratio;
        curr_deadwheel_encoder[2] = m_x.getCurrentPosition();

        packet.put("det ly", curr_deadwheel_encoder[0] - last_deadwheel_encoder[0]);
        packet.put("det ry", curr_deadwheel_encoder[1] - last_deadwheel_encoder[1]);

        heading = spin_ratio * (curr_deadwheel_encoder[0] + curr_deadwheel_encoder[1]) / 2;
        double det_heading = MyMath.clipAngleRadian(heading - last_heading);

        last_heading = heading;
        x_cm_raw = -(curr_deadwheel_encoder[2] - last_deadwheel_encoder[2]) * distance_ratio * x_ratio;
        y_cm_raw = -(double)((curr_deadwheel_encoder[1] - curr_deadwheel_encoder[0]) - (last_deadwheel_encoder[1] - last_deadwheel_encoder[0])) / 2 * distance_ratio * y_ratio;
        double det_x = x_cm_raw - det_heading * forward_offset;
        double det_y = y_cm_raw;

        for(int i = 0; i < 3; i++){
            last_deadwheel_encoder[i] = curr_deadwheel_encoder[i];
        }

        //TODO Add @det_heading to below to see whether it will be better
        x_cm += Math.cos(heading - det_heading / 2) * det_x - Math.sin(heading - det_heading / 2) * det_y;
        y_cm += Math.sin(heading - det_heading / 2) * det_x + Math.cos(heading - det_heading / 2) * det_y;

        curr_pos.set(x_cm , y_cm , heading);
    }

    public void move(double x, double y, double r){
        lf.setPower(x + y + r);
        lb.setPower(-x + y + r);
        rf.setPower(x - y + r);
        rb.setPower(-x - y + r);
    }
}
