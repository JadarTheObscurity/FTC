package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RingPipeline;
import org.firstinspires.ftc.teamcode.util.TowerPipeline;
import org.firstinspires.ftc.teamcode.util.Webcam;
import org.opencv.core.Point;

@Config
@TeleOp(name = "Webcam Tower Tuner", group = "Test")
@Disabled
public class Webcam_Ring extends OpMode {
    FtcDashboard dashboard;
    RingPipeline pipeline = new RingPipeline();
    Webcam webcam;

    public static int pipline_stage = 0;
    public static int min_h = 100;
    public static int max_h = 130;
    public static int min_s = 30;
    public static int max_s = 255;
    public static int min_v = 230;
    public static int max_v = 255;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        webcam = new Webcam(hardwareMap, new Point(640, 480));
        webcam.setPipeline(pipeline);
        webcam.startStreaming(dashboard);
    }

    @Override
    public void loop() {
        TowerPipeline.curr_stage =pipline_stage;
        pipeline.setYCrCbThreshold(min_h, max_h, min_s, max_s, min_v, max_v);
    }
}
