package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.HSV_threshold;
import org.firstinspires.ftc.teamcode.util.RingPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

@Config
@TeleOp(name = "Webcam Ring Tuner", group = "Test")
//@Disabled
public class Webcam_Ring extends OpMode {
    FtcDashboard dashboard;
    RingPipeline pipeline = new RingPipeline();
    Webcam webcam;
    TelemetryPacket packet = new TelemetryPacket();

    public static int pipline_stage = 0;

    public static HSV_threshold threshold = RingPipeline.threshold.copy();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        webcam = new Webcam(hardwareMap, "Webcam Ring", new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.startStreaming(dashboard);
    }

    @Override
    public void loop() {
        packet.put("white pixel number", pipeline.white_pixel);
        packet.put("Ring Status", pipeline.ringStatus);
        dashboard.sendTelemetryPacket(packet);
        RingPipeline.curr_stage = pipline_stage;
        pipeline.setHSVThreshold(threshold);
    }
}
