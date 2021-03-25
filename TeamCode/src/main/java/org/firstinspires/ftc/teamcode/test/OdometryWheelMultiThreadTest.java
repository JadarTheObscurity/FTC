package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.List;

@TeleOp(group = "Test")
@Disabled
public class OdometryWheelMultiThreadTest extends OpMode {
    static DcMotorEx lf, lb, rf, rb;
    static FtcDashboard dashboard;
    static TelemetryPacket packet = new TelemetryPacket();
    static Canvas fieldOverlay;
    static DcMotorEx m_ly, m_ry, m_x;
    static List<LynxModule> allHubs;
    OdometryWheel ow = new OdometryWheel();
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        fieldOverlay = packet.fieldOverlay();
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
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

        ow.start();
    }

    @Override
    public void loop() {
        double raw_x = gamepad1.left_stick_x;
        double raw_y = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;
        double x = raw_x;//raw_x * Math.cos(heading) + raw_y * Math.sin(heading);
        double y = -raw_y;//-raw_x * Math.sin(heading) + raw_y * Math.cos(heading);
        double ratio = 0.7;
        move(ratio * x, ratio * y, r * ratio);
//        computeCoordinate();
        telemetry.update();
    }

    @Override
    public void stop() {
        ow.interrupt();
    }

    public static double pi = 3.14159;
    public static double cmTOInch = 0.393701;
    /**
     *Dead wheel
     *all measured in cm
     */


//    void computeCoordinate(){
//        for (LynxModule module : allHubs) {
//            module.clearBulkCache();
//        }
//        heading = spin_ratio * (m_ly.getCurrentPosition() + m_ry.getCurrentPosition()) / 2;
//        double det_heading = heading - last_heading;
//        last_heading = heading;
//        x_cm_raw = -(m_x.getCurrentPosition() * distance_ratio);
//        y_cm_raw = (double)(m_ry.getCurrentPosition() - m_ly.getCurrentPosition()) / 2 * distance_ratio;
//        double det_x = x_cm_raw - last_x_cm_raw + det_heading * x_correction_ratio;
//        double det_y = y_cm_raw - last_y_cm_raw ;
//
//        x_cm += Math.cos(heading) * det_x - Math.sin(heading) * det_y;
//        y_cm += Math.sin(heading) * det_x + Math.cos(heading) * det_y;
//        last_x_cm_raw = x_cm_raw;
//        last_y_cm_raw = y_cm_raw;
//
//        packet = new TelemetryPacket();
//        packet.put("ly", m_ly.getCurrentPosition());
//        packet.put("ry", m_ry.getCurrentPosition());
//        packet.put("x", m_x.getCurrentPosition());
//        packet.put("angle", Math.toDegrees(heading));
//
//        packet.put("x_cm_raw", x_cm_raw);
//        packet.put("y_cm_raw", y_cm_raw);
//        packet.put("det_x", det_x);
//        packet.put("det_y", det_y);
//        packet.put("x_cm", x_cm);
//        packet.put("y_cm", y_cm);
//        packet.put("det_heading", det_heading);
//        packet.put("correction", det_heading * x_correction_ratio);
//        fieldOverlay = packet.fieldOverlay();
//        Pose2d currentPose = new Pose2d(x_cm * cmTOInch,y_cm * cmTOInch, -heading);
//        fieldOverlay.setStroke("#3F51B5");
//        DashboardUtil.drawRobot(fieldOverlay, currentPose);
//        dashboard.sendTelemetryPacket(packet);
//    }

    static double heading = 0;
    static double last_heading = 0;
    static double x_cm = 0;
    static double y_cm = 0;
    static double x_cm_raw = 0;
    static double y_cm_raw = 0;
    static double last_x_cm_raw = 0;
    static double last_y_cm_raw = 0;

    private static class OdometryWheel extends Thread{
        public double dead_wheel_diameter = 5.7;
        public double lateral_distance = 37;
        public double forward_offset = 6;
        public double encoder_cpr = 1460;
        public double cpr_to_rad = 2 * pi / encoder_cpr;
        public double distance_ratio = 0.0128;//dead_wheel_diameter/ 2 * cpr_to_rad;
        public double spin_ratio = dead_wheel_diameter / lateral_distance * cpr_to_rad * 1.02;
        public double x_correction_ratio = dead_wheel_diameter / 2 / forward_offset * cpr_to_rad;
        public void run(){
            try
            {
                while (!isInterrupted())
                {
                    for (LynxModule module : allHubs) {
                        module.clearBulkCache();
                    }
                    heading = spin_ratio * (m_ly.getCurrentPosition() + m_ry.getCurrentPosition()) / 2;
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
                    Pose2d currentPose = new Pose2d(x_cm * cmTOInch,y_cm * cmTOInch, -heading);
                    fieldOverlay.setStroke("#3F51B5");
                    DashboardUtil.drawRobot(fieldOverlay, currentPose);
                    dashboard.sendTelemetryPacket(packet);
                    Thread.sleep(10);
                }
            }
            catch (InterruptedException e) {System.out.println(e.toString());}

        }
    }

    public void move(double x, double y, double r){
        lf.setPower((x + y  + r));
        lb.setPower((-x + y + r));
        rf.setPower((x - y + r));
        rb.setPower((-x - y + r));
    }

}
